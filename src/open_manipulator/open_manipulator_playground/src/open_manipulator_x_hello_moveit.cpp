// Copyright 2024 ROBOTIS CO., LTD.
// Author: Sungho Woo, Wonho Yun, Woojin Wie

#include "open_manipulator_playground/open_manipulator_x_hello_moveit.h"

#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "pose_cycler",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("pose_cycler");

  // Create the MoveIt MoveGroup Interface for the "arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // --- 1. Define a List of Named Poses from your SRDF ---
  // These names MUST match the "group_state" names in your SRDF file
  std::vector<std::string> target_pose_names = {
    "home",
    "ready",
    "pick",
    "place"
  };

  RCLCPP_INFO(logger, "Starting named pose cycle loop...");

  // --- 2. Cycle Through Poses ---
  while (rclcpp::ok()) {
    
    for (const auto& pose_name : target_pose_names) {
      if (!rclcpp::ok()) break;

      RCLCPP_INFO(logger, "Planning for Named Target: %s", pose_name.c_str());

      // Set the named target (this looks up the values in your SRDF)
      move_group_interface.setNamedTarget(pose_name);

      // Plan
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = static_cast<bool>(move_group_interface.plan(my_plan));

      // Execute
      if (success) {
        RCLCPP_INFO(logger, "Plan successful, executing...");
        move_group_interface.execute(my_plan);
        
        // Wait 2 seconds before the next move
        std::this_thread::sleep_for(std::chrono::seconds(2));
      } else {
        RCLCPP_ERROR(logger, "Planning failed for target: %s", pose_name.c_str());
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}