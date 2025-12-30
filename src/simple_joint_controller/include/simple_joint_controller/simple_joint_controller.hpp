#ifndef SIMPLE_JOINT_CONTROLLER_HPP_
#define SIMPLE_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace simple_joint_controller
{
class SimpleJointController : public controller_interface::ControllerInterface
{
public:
  SimpleJointController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  
  // Current state
  std::vector<double> current_positions_;
  std::vector<double> desired_positions_;
  
  // Trajectory tracking
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> rt_trajectory_buffer_;
  size_t trajectory_index_;
  rclcpp::Time trajectory_start_time_;
  bool is_executing_;

  // ROS subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
};

}  // namespace simple_joint_controller

#endif  // SIMPLE_JOINT_CONTROLLER_HPP_
