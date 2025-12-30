#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

// =========================================================================================
//                                  USER CONFIGURATION
// =========================================================================================
// TABLE: Large surface centered on (0,0)
const double TABLE_DIM_X = 2.0; 
const double TABLE_DIM_Y = 2.0; 
const double TABLE_DIM_Z = 0.05; 
const double TABLE_POS_Z = -0.025; // Surface at Z=0.0

// CUBE: Target Object
const double CUBE_SIZE = 0.025; 
// Matches SRDF "pick" (Joint1 ~ 90 deg -> Y axis)
const double CUBE_X = 0.0; 
const double CUBE_Y = 0.20;
const double CUBE_Z = 0.02; // Slightly above 0 to prevent "table collision" noise
// =========================================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ex11_planning_scene_attach",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ex11_planning_scene_attach");

  // 1. Setup Interfaces
  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;

  MoveGroupInterface arm_group(node, "arm");
  MoveGroupInterface gripper_group(node, "gripper");
  PlanningSceneInterface planning_scene_interface;

  // 2. Setup Scene
  // --------------------------------------------------------
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = arm_group.getPlanningFrame();
  table.id = "table";
  shape_msgs::msg::SolidPrimitive table_prim;
  table_prim.type = table_prim.BOX;
  table_prim.dimensions = {TABLE_DIM_X, TABLE_DIM_Y, TABLE_DIM_Z};
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.0; table_pose.position.y = 0.0; table_pose.position.z = TABLE_POS_Z;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_prim);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = arm_group.getPlanningFrame();
  cube.id = "cube";
  shape_msgs::msg::SolidPrimitive cube_prim;
  cube_prim.type = cube_prim.BOX;
  cube_prim.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = CUBE_X; cube_pose.position.y = CUBE_Y; cube_pose.position.z = CUBE_Z;
  cube_pose.orientation.w = 1.0;
  cube.primitives.push_back(cube_prim);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;

  RCLCPP_INFO(logger, "Adding Objects to Scene...");
  planning_scene_interface.applyCollisionObjects({table, cube});
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 3. Execution Sequence
  // --------------------------------------------------------

  // Step 1: Home
  RCLCPP_INFO(logger, "1. Moving Home...");
  arm_group.setNamedTarget("home");
  arm_group.move();

  // Step 2: Open Gripper
  RCLCPP_INFO(logger, "2. Opening Gripper...");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // Step 3: Move to Pick
  RCLCPP_INFO(logger, "3. Moving to Pick Pose...");
  arm_group.setNamedTarget("pick");
  arm_group.move();

  // Step 4: ATTACH FIRST (Crucial Fix!)
  // We attach the object BEFORE closing the gripper. 
  // This updates the Allowed Collision Matrix (ACM) so the gripper can touch the cube without error.
  RCLCPP_INFO(logger, "4. Attaching Object...");
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "link5";
  attached_object.object = cube;
  attached_object.object.operation = attached_object.object.ADD;
  attached_object.touch_links = {"gripper_left_link", "gripper_right_link", "link5"};
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 5: Close Gripper (Now safe to move)
  RCLCPP_INFO(logger, "5. Closing Gripper...");
  gripper_group.setNamedTarget("close");
  gripper_group.move();

  // Step 6: Lift / Home (Crucial Fix!)
  // We MUST go up before going to "place" to avoid dragging the cube through the table.
  RCLCPP_INFO(logger, "6. Lifting to Home...");
  arm_group.setNamedTarget("home");
  if (!arm_group.move()) {
    RCLCPP_ERROR(logger, "Failed to lift! Stopping.");
    return 1;
  }

  // Step 7: Move to Place
  RCLCPP_INFO(logger, "7. Moving to Place Pose...");
  arm_group.setNamedTarget("place");
  if (!arm_group.move()) {
    RCLCPP_ERROR(logger, "Failed to reach place pose!");
  }

  // Step 8: Open Gripper
  RCLCPP_INFO(logger, "8. Releasing...");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // Step 9: Detach
  RCLCPP_INFO(logger, "9. Detaching Object...");
  attached_object.object.operation = attached_object.object.REMOVE;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 10: Lift & Home
  RCLCPP_INFO(logger, "10. Returning Home...");
  arm_group.setNamedTarget("home");
  arm_group.move();
  
  RCLCPP_INFO(logger, "Demo Complete!");
  rclcpp::shutdown();
  return 0;
}