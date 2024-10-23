#ifndef RACECAR_SYSTEM_INTERFACE_HPP
#define RACECAR_SYSTEM_INTERFACE_HPP

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "serial_communicator.hpp"  // Include the SerialCommunicator
#include <string>

namespace racecar_system
{

class RacecarSystemHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RacecarSystemHardware();
  ~RacecarSystemHardware();

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  // Variables to hold the state and command values
  double servo_position_command_;
  double servo_position_state_;
  double servo_velocity_state_;
  double servo_velocity_command_;
  
  // Front Left Wheel
  double front_left_wheel_position_state_;
  double front_left_wheel_velocity_state_;
  double front_left_wheel_velocity_command_;

  // Front Right Wheel
  double front_right_wheel_position_state_;
  double front_right_wheel_velocity_state_;
  double front_right_wheel_velocity_command_;

  double traction_velocity_command_;
  double traction_velocity_state_;
  double traction_position_state_;

  rclcpp::Time last_time_;
  rclcpp::Clock clock_;

  // Serial communication
  SerialCommunicator serial_;

  std::string port_;
  int baudrate_;
};

}  // namespace racecar_system

#endif  // RACECAR_SYSTEM_INTERFACE_HPP
