#include "simple_joint_controller/simple_joint_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/logging.hpp"

namespace simple_joint_controller
{

SimpleJointController::SimpleJointController()
: controller_interface::ControllerInterface(),
  trajectory_index_(0),
  is_executing_(false)
{
}

controller_interface::CallbackReturn SimpleJointController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SimpleJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/position");
  }

  return config;
}

controller_interface::InterfaceConfiguration
SimpleJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/position");
  }

  return config;
}

controller_interface::CallbackReturn SimpleJointController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  current_positions_.resize(joint_names_.size(), 0.0);
  desired_positions_.resize(joint_names_.size(), 0.0);

  // Create subscriber for joint trajectory
  trajectory_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/simple_trajectory", 10,
    std::bind(&SimpleJointController::trajectory_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), 
    "Configured SimpleJointController with %zu joints", joint_names_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleJointController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read current positions from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    current_positions_[i] = state_interfaces_[i].get_value();
    desired_positions_[i] = current_positions_[i];
  }

  trajectory_index_ = 0;
  is_executing_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "Activated SimpleJointController");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleJointController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Hold current position on deactivation
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(current_positions_[i]);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Deactivated SimpleJointController");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleJointController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Get trajectory from realtime buffer
  trajectory_msg_ = *rt_trajectory_buffer_.readFromRT();

  if (!trajectory_msg_ || trajectory_msg_->points.empty())
  {
    // No trajectory, hold position
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(desired_positions_[i]);
    }
    return controller_interface::return_type::OK;
  }

  // Read current state
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    current_positions_[i] = state_interfaces_[i].get_value();
  }

  // Execute trajectory
  if (!is_executing_)
  {
    trajectory_start_time_ = time;
    trajectory_index_ = 0;
    is_executing_ = true;
  }

  auto time_from_start = time - trajectory_start_time_;
  
  // Find current segment
  while (trajectory_index_ < trajectory_msg_->points.size() - 1 &&
         time_from_start > rclcpp::Duration(trajectory_msg_->points[trajectory_index_ + 1].time_from_start))
  {
    trajectory_index_++;
  }

  if (trajectory_index_ >= trajectory_msg_->points.size())
  {
    // Trajectory finished
    is_executing_ = false;
    return controller_interface::return_type::OK;
  }

  // Linear interpolation
  const auto & current_point = trajectory_msg_->points[trajectory_index_];
  
  if (trajectory_index_ < trajectory_msg_->points.size() - 1)
  {
    const auto & next_point = trajectory_msg_->points[trajectory_index_ + 1];
    auto segment_start = rclcpp::Duration(current_point.time_from_start);
    auto segment_end = rclcpp::Duration(next_point.time_from_start);
    auto segment_duration = segment_end - segment_start;

    if (segment_duration.seconds() > 0.0001)
    {
      double alpha = (time_from_start - segment_start).seconds() / segment_duration.seconds();
      alpha = std::clamp(alpha, 0.0, 1.0);

      for (size_t i = 0; i < joint_names_.size() && i < current_point.positions.size(); ++i)
      {
        desired_positions_[i] = current_point.positions[i] +
          alpha * (next_point.positions[i] - current_point.positions[i]);
      }
    }
  }
  else
  {
    // Last point
    for (size_t i = 0; i < joint_names_.size() && i < current_point.positions.size(); ++i)
    {
      desired_positions_[i] = current_point.positions[i];
    }
  }

  // Write commands
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(desired_positions_[i]);
  }

  return controller_interface::return_type::OK;
}

void SimpleJointController::trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received trajectory with %zu points", msg->points.size());
  
  if (msg->points.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Empty trajectory received");
    return;
  }

  rt_trajectory_buffer_.writeFromNonRT(msg);
  is_executing_ = false;  // Will restart on next update
}

}  // namespace simple_joint_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  simple_joint_controller::SimpleJointController,
  controller_interface::ControllerInterface)
