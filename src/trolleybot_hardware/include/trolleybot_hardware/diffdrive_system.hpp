#ifndef TROLLEYBOT_HARDWARE__DIFFDRIVE_SYSTEM_HPP_
#define TROLLEYBOT_HARDWARE__DIFFDRIVE_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace trolleybot_hardware
{

/// ros2_control SystemInterface for the DIY differential-drive base.
///
/// Phase 2 skeleton: the read()/write() bodies currently dead-reckon from the
/// commanded values so the stack runs end-to-end without hardware. Replace the
/// marked sections with the serial/CAN protocol to the real motor controller.
class DiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint storage (indexed as in info_.joints).
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<std::string> command_types_;  // "velocity" or "position"

  // Connection parameters (from the <hardware> block in the URDF).
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};
  int encoder_counts_per_rev_{2048};
  double wheel_radius_{0.10};
};

}  // namespace trolleybot_hardware

#endif  // TROLLEYBOT_HARDWARE__DIFFDRIVE_SYSTEM_HPP_
