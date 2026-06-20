#include "trolleybot_hardware/diffdrive_system.hpp"

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace trolleybot_hardware
{

hardware_interface::CallbackReturn DiffDriveSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional connection parameters from the URDF <hardware> block.
  auto get_param = [this](const std::string & key, const std::string & fallback) {
    auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? it->second : fallback;
  };
  serial_port_ = get_param("serial_port", serial_port_);
  baud_rate_ = std::stoi(get_param("baud_rate", std::to_string(baud_rate_)));
  encoder_counts_per_rev_ =
    std::stoi(get_param("encoder_counts_per_rev", std::to_string(encoder_counts_per_rev_)));
  wheel_radius_ = std::stod(get_param("wheel_radius", std::to_string(wheel_radius_)));

  const size_t n = info_.joints.size();
  hw_commands_.assign(n, 0.0);
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  command_types_.resize(n);

  for (size_t i = 0; i < n; ++i)
  {
    if (info_.joints[i].command_interfaces.empty())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystem"),
        "Joint '%s' has no command interface.", info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    command_types_[i] = info_.joints[i].command_interfaces[0].name;
  }

  // TODO(phase2): open the serial/CAN connection to the motor controller here
  //               (serial_port_, baud_rate_) and verify a handshake.

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, command_types_[i], &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(phase2): command zero velocity to the controller and close the link.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(phase2): read encoder ticks over serial and convert to position/velocity.
  // Placeholder dead-reckoning so the stack runs without hardware:
  const double dt = period.seconds();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (command_types_[i] == hardware_interface::HW_IF_VELOCITY)
    {
      hw_velocities_[i] = hw_commands_[i];
      hw_positions_[i] += hw_velocities_[i] * dt;
    }
    else  // position-commanded joint (e.g. lift)
    {
      const double prev = hw_positions_[i];
      hw_positions_[i] = hw_commands_[i];
      hw_velocities_[i] = dt > 0.0 ? (hw_positions_[i] - prev) / dt : 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(phase2): serialize hw_commands_ and send to the motor controller.
  return hardware_interface::return_type::OK;
}

}  // namespace trolleybot_hardware

PLUGINLIB_EXPORT_CLASS(
  trolleybot_hardware::DiffDriveSystem, hardware_interface::SystemInterface)
