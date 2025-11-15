#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "scara_robot_moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("scara_robot_moveit_node");

  // Create MoveGroupInterface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "scara_arm");

  // Set planning parameters
  move_group_interface.setPlanningTime(20.0);  // Increased planning time
  move_group_interface.setNumPlanningAttempts(20);  // More attempts
  move_group_interface.setGoalTolerance(0.01);  // Set goal tolerance
  move_group_interface.setMaxVelocityScalingFactor(0.5);  // Slow down for debugging
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  // Set start state to current state (best-effort). If no joint_states are available
  // MoveIt will fall back to the model/default state. We'll handle missing state below.
  move_group_interface.setStartStateToCurrentState();

  // Print current pose for debugging
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current end effector pose: x=%.3f, y=%.3f, z=%.3f", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Set a simpler target pose first - try joint space goal instead
  RCLCPP_INFO(logger, "Trying joint space goal first...");
  
  // Try joint-space planning only if we have a recent robot state (joint values).
  auto current_joint_values = move_group_interface.getCurrentJointValues();
  bool did_joint_plan = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (!current_joint_values.empty()) {
    RCLCPP_INFO(logger, "Current joint values:");
    for (size_t i = 0; i < current_joint_values.size(); ++i) {
      RCLCPP_INFO(logger, "  Joint %zu: %.3f", i, current_joint_values[i]);
    }

    // Set a small joint movement (first joint) and plan.
    std::vector<double> joint_goal = current_joint_values;
    joint_goal[0] += 0.1;  // Small movement of first joint
    move_group_interface.setJointValueTarget(joint_goal);

    auto success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(logger, "Joint-space planning succeeded. (NOTE: this node only plans; not executing trajectories)");
      // Print some simple info about the computed plan
      try {
        RCLCPP_INFO(logger, "Planned trajectory contains %zu points", plan.trajectory_.joint_trajectory.points.size());
      } catch (...) {
        RCLCPP_INFO(logger, "Planned trajectory info not available to print");
      }
      did_joint_plan = true;
    } else {
      RCLCPP_WARN(logger, "Joint-space planning failed; will try a simple pose goal as fallback");
    }
  } else {
    RCLCPP_WARN(logger, "No recent joint state available; skipping joint-space plan and trying pose goal");
  }

  if (!did_joint_plan) {
    // Try pose goal as fallback
    RCLCPP_INFO(logger, "Trying a simple pose goal...");
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.1;  // Simple reachable target in robot frame
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.1;

    move_group_interface.setPoseTarget(target_pose);
    auto success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(logger, "Pose planning succeeded. (NOTE: this node only plans; not executing trajectories)");
      try {
        RCLCPP_INFO(logger, "Planned trajectory contains %zu points", plan.trajectory_.joint_trajectory.points.size());
      } catch (...) {
        RCLCPP_INFO(logger, "Planned trajectory info not available to print");
      }
    } else {
      RCLCPP_ERROR(logger, "All planning attempts failed!");
    }
  }

  rclcpp::shutdown();
  return 0;
}