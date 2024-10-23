#include "../include/racecar_system_hardware.hpp"
#include "../include/racecar_motor_utils.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cerrno>

namespace racecar_system
{

RacecarSystemHardware::RacecarSystemHardware() : clock_(RCL_SYSTEM_TIME) {}

RacecarSystemHardware::~RacecarSystemHardware() {}

hardware_interface::return_type RacecarSystemHardware::configure(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::BaseInterface<hardware_interface::SystemInterface>::configure(info) !=
      hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Initialize variables
  servo_position_state_ = 0.0;
  servo_velocity_state_ = 0.0; // from twist comes only angular velocity in rad/s
  servo_position_command_ = 0.1;
  servo_velocity_command_ = 0.0;
 front_left_wheel_position_state_ = 0.0;
 front_left_wheel_velocity_state_ = 0.0;
 front_left_wheel_velocity_command_ = 0.0;

// Front Right Wheel
 front_right_wheel_position_state_ = 0.0;
 front_right_wheel_velocity_state_ = 0.0;
 front_right_wheel_velocity_command_= 0.0;

  traction_velocity_command_ = 0.0;
  traction_velocity_state_ = 0.0;
  traction_position_state_ = 0.0;

  // Read parameters from the hardware info
  port_ = info_.hardware_parameters.at("port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  // Initialize the racecar hardware
  if (serial_.openPort(port_) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RacecarSystemHardware"), "Failed to open serial port");
    return hardware_interface::return_type::ERROR;
  }

  if (serial_.configurePort(baudrate_, 8, 'N', 1) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RacecarSystemHardware"), "Failed to set serial port options");
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "Racecar hardware interface configured.");

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RacecarSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Traction joint state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("traction_joint", hardware_interface::HW_IF_VELOCITY, &traction_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("traction_joint", hardware_interface::HW_IF_POSITION, &traction_position_state_));

  // Front Left Wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_wheel_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &front_left_wheel_position_state_));

  // Front Right Wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_wheel_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &front_right_wheel_position_state_));
    
  // Steering joint state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface("steering_joint", hardware_interface::HW_IF_VELOCITY, &servo_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("steering_joint", hardware_interface::HW_IF_POSITION, &servo_position_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RacecarSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Traction joint command interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface("traction_joint", hardware_interface::HW_IF_VELOCITY, &traction_velocity_command_));

  // Steering Joint Command Interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface("steering_joint", hardware_interface::HW_IF_POSITION, &servo_position_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("steering_joint", hardware_interface::HW_IF_VELOCITY, &servo_velocity_command_));

  return command_interfaces;
}

hardware_interface::return_type RacecarSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "Starting racecar hardware interface...");

  // Set initial states
  traction_velocity_state_ = traction_velocity_command_;
  traction_position_state_ = 0.0;

  front_left_wheel_velocity_state_ = servo_velocity_command_;
  front_left_wheel_position_state_ = servo_velocity_command_; 

  front_right_wheel_velocity_state_ = servo_velocity_command_;
  front_right_wheel_position_state_ = servo_velocity_command_; 

  servo_position_state_ = servo_position_command_;
  servo_velocity_state_ = servo_velocity_command_;

  // Initialize last_time_
  last_time_ = clock_.now();

  RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "Racecar hardware interface started.");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RacecarSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "Stopping racecar hardware interface...");

  // Stop the car by sending zero commands
  servo_velocity_state_ = 0.0;
  servo_position_command_ = 0.0;
  servo_velocity_command_= 0.0;

  traction_velocity_command_ = 0.0;
  traction_velocity_state_ = 0.0;
  write();

  RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "Racecar hardware interface stopped.");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RacecarSystemHardware::read()
{
  rclcpp::Time current_time = clock_.now();
  double dt = (current_time - last_time_).seconds();

  //const double max_steering_angle = M_PI / 4;  // +/- 45 degrees
  
  if (dt <= 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("RacecarSystemHardware"), "Non-positive dt encountered: %f", dt);
    dt = 0.01;  // Assign a small positive value or skip the update
  }
  last_time_ = current_time;


  traction_position_state_ += traction_velocity_state_ * dt;

  traction_velocity_state_ = traction_velocity_command_;
  
// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: traction_position_state_: %f", traction_position_state_);
// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: traction_velocity_state_: %f", traction_velocity_state_);
// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: traction_velocity_command_: %f", traction_velocity_command_);
  // Update steering joint state
  

  front_left_wheel_velocity_state_ = servo_velocity_command_;
  front_left_wheel_position_state_ = -servo_velocity_command_; 

  front_right_wheel_velocity_state_ = servo_velocity_command_;
  front_right_wheel_position_state_ = -servo_velocity_command_; 

  servo_velocity_state_ = -servo_velocity_command_;
  
  servo_position_state_ = servo_velocity_state_;


// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: servo_position_state_: %f", servo_position_state_);
// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: servo_velocity_state_: %f", servo_velocity_state_);
// RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: servo_velocity_command_: %f", servo_velocity_command_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RacecarSystemHardware::write()
{
  //RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: traction_velocity_command_: %f", traction_velocity_command_);
  //RCLCPP_INFO(rclcpp::get_logger("RacecarSystemHardware"), "READ: servo_velocity_command_: %f", servo_velocity_command_);
  uint16_t motor_pwm = racecar_utils::calculateMotorPWM2(traction_velocity_command_);

  uint16_t servo_pwm = racecar_utils::calculateServoPWM(servo_velocity_command_);

  // Send commands to the hardware
  serial_.sendCmd(motor_pwm, servo_pwm);

  return hardware_interface::return_type::OK;
}

}  // namespace racecar_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(racecar_system::RacecarSystemHardware, hardware_interface::SystemInterface)
