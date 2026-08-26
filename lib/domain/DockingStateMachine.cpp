#include <cmath>

#include "DockingStateMachine.h"

namespace fal {

namespace {
float absf(float v) { return v < 0 ? -v : v; }
} // namespace

DockingStateMachine::DockingStateMachine(IDrive& drive, IClamp& clamp, ILimitSwitches& limits,
                                         IOdometry& odom, SafetyMonitor& safety, IClock& clock,
                                         const DockingConfig& cfg)
    : drive_(drive), clamp_(clamp), limits_(limits), odom_(odom), safety_(safety),
      clock_(clock), cfg_(cfg) {}

void DockingStateMachine::enter(DockState s) {
  state_ = s;
  state_since_ms_ = clock_.millis();
}

void DockingStateMachine::applyMove(const DriveCommand& cmd, uint32_t now) {
  drive_.move(cmd);
  odom_.update(cmd, now);  // integrate the motion we just commanded
}

void DockingStateMachine::applyStop(uint32_t now) {
  drive_.stop();
  odom_.update(DriveCommand{}, now);  // keep the time base advancing; no motion
}

void DockingStateMachine::toFault(const char* reason) {
  last_reason_ = reason;
  confirmed_ = false;
  enter(DockState::Fault);
  drive_.stop();
  drive_.brake(true);
  drive_.enable(false);
  clamp_.stop();
}

void DockingStateMachine::handleCommand(Command c) {
  switch (c) {
    case Command::Dock:
      if (state_ == DockState::Idle) {
        drive_.brake(false);
        drive_.enable(true);
        odom_.reset();
        confirmed_ = false;
        x_far_found_ = false;
        y_phase_ = 0;
        confirm_holding_ = false;
        enter(DockState::Approach);
      }
      break;
    case Command::Unclamp:
      if (state_ == DockState::Clamped) enter(DockState::Unclamping);
      break;
    case Command::Abort:
      drive_.stop();
      drive_.brake(false);
      drive_.enable(true);
      clamp_.stop();
      safety_.clearEstopLatch();
      confirmed_ = false;
      enter(DockState::Idle);
      break;
    case Command::Status:
    case Command::None:
    default:
      break;
  }
}

void DockingStateMachine::update(const bool p[kNumCorners]) {
  const uint32_t now = clock_.millis();
  const bool fl = p[0], fr = p[1];  // front pair (leading edge, perpendicular to motion)

  // Global interlock: any safe-stop condition forces FAULT and stays there.
  if (safety_.safeStopRequired() && state_ != DockState::Fault) {
    toFault("safety");
    return;
  }

  switch (state_) {
    case DockState::Idle:
      applyStop(now);
      clamp_.stop();
      break;

    case DockState::Approach:
      applyMove({cfg_.approach_speed, 0.0f, 0.0f}, now);
      if (fl || fr) {
        enter(DockState::Orient);  // a front corner reached the near edge
      } else if (now - state_since_ms_ > cfg_.approach_timeout_ms) {
        toFault("approach timeout");
      }
      break;

    case DockState::Orient:
      if (fl && fr) {
        applyStop(now);
        x_near_ = odom_.pose().x_mm;  // both leading corners at the near edge
        x_far_found_ = false;
        x_lo_found_ = false;
        enter(DockState::CenterX);
      } else {
        // Rotate to bring the lagging leading corner onto the edge. The sign is
        // configuration, not a constant -- see DockingConfig::rotate_dir.
        const float w = cfg_.rotate_dir *
                        ((fl && !fr) ? -cfg_.rotate_speed : cfg_.rotate_speed);
        applyMove({0.0f, 0.0f, w}, now);
        if (now - state_since_ms_ > cfg_.orient_timeout_ms) toFault("orient timeout");
      }
      break;

    case DockState::CenterX:
      if (!x_far_found_) {
        applyMove({cfg_.centre_speed, 0.0f, 0.0f}, now);  // advance to the far edge

        if (!fl && !fr) {
          x_far_ = odom_.pose().x_mm;
          if (cfg_.centre_opposed_pairs) {
            // Never seen the trailing pair covered means the platform is not
            // deeper than the sensor span, or a trailing sensor is dead. Either
            // way the opposed midpoint would be fiction, and silently falling
            // back to the leading-pair method would hide a real fault.
            if (!x_lo_found_) {
              toFault("trailing pair never covered — cannot centre on opposed edges");
              return;
            }
            x_target_ = 0.5f * (x_lo_ + x_far_) - cfg_.front_offset_mm;
          } else {
            x_target_ = 0.5f * (x_near_ + x_far_) - cfg_.front_offset_mm;
          }
          x_far_found_ = true;
        } else {
          // STILL UNDER THE PLATFORM. The opposed near-edge mark is taken here
          // and only here -- while the leading pair is still covered -- so it
          // can never be captured by the same tick that closes the seek. That
          // ordering is the geometry, not defensiveness: the trailing pair
          // reaches the near edge a full (depth - sensor span) before the
          // leading pair reaches the far one, and two marks at one position
          // would bisect to nothing.
          if (cfg_.centre_opposed_pairs && !x_lo_found_ && p[2] && p[3]) {
            x_lo_ = odom_.pose().x_mm;
            x_lo_found_ = true;
          }
          if (now - state_since_ms_ > cfg_.center_timeout_ms) {
            toFault("centerX seek timeout");
          }
        }
      } else {
        const float err = x_target_ - odom_.pose().x_mm;
        if (absf(err) <= cfg_.centre_tol_mm) {
          applyStop(now);
          y_phase_ = 0;
          // Straight to Confirm when the platform cannot travel laterally --
          // see DockingConfig::centre_lateral. Confirm still demands all four
          // corners, so a base parked badly on the uncorrectable axis is caught
          // there rather than silently accepted.
          enter(cfg_.centre_lateral ? DockState::CenterY : DockState::Confirm);
        } else {
          const float dir = err > 0 ? 1.0f : -1.0f;
          applyMove({dir * cfg_.centre_speed, 0.0f, 0.0f}, now);
          if (now - state_since_ms_ > cfg_.center_timeout_ms) toFault("centerX goto timeout");
        }
      }
      break;

    case DockState::CenterY: {
      const bool left = p[0] && p[2];   // FL & RL over the board
      const bool right = p[1] && p[3];  // FR & RR over the board
      if (y_phase_ == 0) {
        applyMove({0.0f, cfg_.centre_speed, 0.0f}, now);  // strafe left to the left edge
        if (!left) {
          y_a_ = odom_.pose().y_mm;
          y_phase_ = 1;
        } else if (now - state_since_ms_ > cfg_.center_timeout_ms) {
          toFault("centerY left timeout");
        }
      } else if (y_phase_ == 1) {
        applyMove({0.0f, -cfg_.centre_speed, 0.0f}, now);  // strafe right to the right edge
        if (!right) {
          y_b_ = odom_.pose().y_mm;
          y_target_ = 0.5f * (y_a_ + y_b_) - cfg_.side_offset_mm;
          y_phase_ = 2;
        } else if (now - state_since_ms_ > cfg_.center_timeout_ms) {
          toFault("centerY right timeout");
        }
      } else {
        const float err = y_target_ - odom_.pose().y_mm;
        if (absf(err) <= cfg_.centre_tol_mm) {
          applyStop(now);
          confirm_holding_ = false;
          enter(DockState::Confirm);
        } else {
          const float dir = err > 0 ? 1.0f : -1.0f;
          applyMove({0.0f, dir * cfg_.centre_speed, 0.0f}, now);
          if (now - state_since_ms_ > cfg_.center_timeout_ms) toFault("centerY goto timeout");
        }
      }
      break;
    }

    case DockState::Confirm: {
      applyStop(now);
      const bool all = p[0] && p[1] && p[2] && p[3];
      if (all) {
        if (!confirm_holding_) {
          confirm_holding_ = true;
          confirm_since_ms_ = now;
        } else if (now - confirm_since_ms_ >= cfg_.confirm_hold_ms) {
          confirmed_ = true;
          enter(DockState::ClampEngage);
        }
      } else {
        confirm_holding_ = false;  // lost coverage — keep waiting
      }
      // ...but not forever. If all four corners never coincide, waiting is not
      // patience, it is a stall the operator cannot distinguish from success.
      if (now - state_since_ms_ > cfg_.confirm_timeout_ms) {
        toFault("confirm timeout — all four corners never coincided");
      }
      break;
    }

    case DockState::ClampEngage:
      applyStop(now);
      // HANDOFF to the external clamp subsystem — only through the safety gate.
      if (confirmed_ && !safety_.safeStopRequired()) {
        clamp_.close(cfg_.clamp_speed);
        limits_.update();
        if (limits_.clampClosed()) {
          clamp_.stop();
          enter(DockState::Clamped);
        } else if (now - state_since_ms_ > cfg_.clamp_timeout_ms) {
          toFault("clamp timeout");
        }
      } else {
        clamp_.stop();  // gate closed — never engage
      }
      break;

    case DockState::Clamped:
      applyStop(now);
      clamp_.stop();
      break;

    case DockState::Unclamping:
      clamp_.open(cfg_.clamp_speed);
      limits_.update();
      if (limits_.clampOpen()) {
        clamp_.stop();
        confirmed_ = false;
        enter(DockState::Idle);
      } else if (now - state_since_ms_ > cfg_.unclamp_timeout_ms) {
        toFault("unclamp timeout");
      }
      break;

    case DockState::Fault:
      drive_.stop();
      drive_.brake(true);
      drive_.enable(false);
      clamp_.stop();
      break;
  }
}

const char* DockingStateMachine::stateName() const {
  switch (state_) {
    case DockState::Idle: return "IDLE";
    case DockState::Approach: return "APPROACH";
    case DockState::Orient: return "ORIENT";
    case DockState::CenterX: return "CENTER_X";
    case DockState::CenterY: return "CENTER_Y";
    case DockState::Confirm: return "CONFIRM";
    case DockState::ClampEngage: return "CLAMP_ENGAGE";
    case DockState::Clamped: return "CLAMPED";
    case DockState::Unclamping: return "UNCLAMPING";
    case DockState::Fault: return "FAULT";
  }
  return "?";
}

} // namespace fal
