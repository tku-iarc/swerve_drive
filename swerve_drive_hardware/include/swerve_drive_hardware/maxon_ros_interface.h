#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "mobile_base_msgs/msg/MotorCmds.hpp"
// #include "mobile_base_msgs/msg/MotorStates.hpp"
#include "swerve_drive_hardware/visibility_control.h"


namespace swerve_drive_hardware
{
  
class MaxonRosInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MaxonRosInterface)

  float sample_rate;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_DRIVE_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the commands for the simulated robot
  std::vector<int> hw_motors_id_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_accelerations_;
  std::vector<double> hw_gear_ratio_;

  // void motorStatesCallback(const mobile_base_msgs::msg::MotorStates::SharedPtr msg);
  // hardware_interface::JointStateInterface jnt_state_interface;
  // hardware_interface::PositionJointInterface jnt_pos_interface;
  // hardware_interface::VelocityJointInterface jnt_vel_interface;
 
  int control_mode;

  // rclcpp::Publisher motor_cmds_pub_;
  // rclcpp::Subscriber motor_states_sub_;
  bool received_states;
  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    ACCELERATION = 3
  };

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;
};

} // namespace name