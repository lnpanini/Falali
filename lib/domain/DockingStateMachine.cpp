#include "DockingStateMachine.h"

namespace tb {

DockingStateMachine::DockingStateMachine(IDrive& drive, IClamp& clamp,
                                         ILimitSwitches& limits, SafetyMonitor& safety,
                                         IClock& clock, const DockingConfig& cfg)
    : drive_(drive), clamp_(clamp), limits_(limits), safety_(safety), clock_(clock),
      cfg_(cfg) {}

void DockingStateMachine::enter(DockState s) {
  state_ = s;
  state_since_ms_ = clock_.millis();
}

void DockingStateMachine::toFault(const char* reason) {
  last_reason_ = reason;
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
        enter(DockState::Entering);
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
      enter(DockState::Idle);
      break;
    case Command::Status:
    case Command::None:
    default:
      break;
  }
}

void DockingStateMachine::update(const AlignmentState& align) {
  const uint32_t now = clock_.millis();

  // Global interlock: any safe-stop condition forces FAULT and stays there.
  if (safety_.safeStopRequired() && state_ != DockState::Fault) {
    toFault("safety");
    return;
  }

  switch (state_) {
    case DockState::Idle:
      drive_.stop();
      clamp_.stop();
      break;

    case DockState::Entering:
      drive_.move({cfg_.enter_speed, 0.0f, 0.0f});
      if (align.under_trolley >= cfg_.enter_under_threshold) {
        enter(DockState::Aligning);
      } else if (now - state_since_ms_ > cfg_.enter_timeout_ms) {
        toFault("enter timeout");
      }
      break;

    case DockState::Aligning: {
      float vy = -cfg_.centre_kp * align.lateral;  // strafe toward centre
      if (vy > cfg_.max_strafe) vy = cfg_.max_strafe;
      if (vy < -cfg_.max_strafe) vy = -cfg_.max_strafe;
      drive_.move({0.0f, vy, 0.0f});
      if (align.clamp_safe) {
        enter(DockState::ReadyToClamp);
      } else if (now - state_since_ms_ > cfg_.align_timeout_ms) {
        toFault("align timeout");
      }
      break;
    }

    case DockState::ReadyToClamp:
      drive_.stop();
      // Only cross into clamping when the safety gate is open right now.
      if (safety_.clampCloseAllowed()) {
        enter(DockState::Clamping);
      } else {
        enter(DockState::Aligning);  // lost the gate — recover alignment
      }
      break;

    case DockState::Clamping:
      // Entry was gated by safety; keep closing until the CLOSED switch trips.
      // Over-current / E-stop are handled by the global safe-stop check above.
      clamp_.close(cfg_.clamp_speed);
      limits_.update();
      if (limits_.clampClosed()) {
        clamp_.stop();
        enter(DockState::Clamped);
      } else if (now - state_since_ms_ > cfg_.clamp_timeout_ms) {
        toFault("clamp timeout");
      }
      break;

    case DockState::Clamped:
      drive_.stop();
      clamp_.stop();
      break;

    case DockState::Unclamping:
      clamp_.open(cfg_.clamp_speed);
      limits_.update();
      if (limits_.clampOpen()) {
        clamp_.stop();
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
    case DockState::Entering: return "ENTERING";
    case DockState::Aligning: return "ALIGNING";
    case DockState::ReadyToClamp: return "READY_TO_CLAMP";
    case DockState::Clamping: return "CLAMPING";
    case DockState::Clamped: return "CLAMPED";
    case DockState::Unclamping: return "UNCLAMPING";
    case DockState::Fault: return "FAULT";
  }
  return "?";
}

} // namespace tb
