#!/usr/bin/env bash
# Helper script to spawn ros2_control controllers and joint_state_broadcaster
# Usage: ./spawn_controllers.sh [controller_manager_namespace]
# Example: ./spawn_controllers.sh /controller_manager

set -euo pipefail
CM_NS=${1:-/controller_manager}

echo "Using controller manager namespace: ${CM_NS}"

echo "Spawning joint_state_broadcaster..."
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager ${CM_NS}

echo "Spawning scara_arm_controller..."
ros2 run controller_manager spawner scara_arm_controller --controller-manager ${CM_NS}

echo "Spawning scara_gripper_controller..."
ros2 run controller_manager spawner scara_gripper_controller --controller-manager ${CM_NS}

echo "Done. Verify controllers with:"
echo "  ros2 service call ${CM_NS}/list_controllers controller_manager_msgs/srv/ListControllers '{}'"
echo "Check /joint_states with: ros2 topic echo /joint_states -n1"
