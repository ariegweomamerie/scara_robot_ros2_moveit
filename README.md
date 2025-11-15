# scara_robot_ros2_moveit
This repository contains a ROS2 workspace for a SCARA robot. It includes the URDF model, MoveIt2 configuration for motion planning, and launch files for simulation and visualization. The robot model is provided with STL meshes and configured with ROS2 Control. The MoveIt2 setup allows for motion planning using OMPL, CHOMP, and STOMP planners.


📋 Table of Contents

    Overview

    Prerequisites

    Workspace Setup

    MoveIt Setup Assistant Guide

    Building the Workspace

    Running the Demo

    Moving Your Robot with MoveIt

    Project Structure

    Troubleshooting


Overview

This project provides a complete ROS2 implementation of a SCARA robot with MoveIt2 integration, including:

    URDF robot model with realistic 3D meshes

    MoveIt2 configuration for motion planning

    ROS2 control interfaces

    RViz visualization configurations

    Ready-to-use launch files

Prerequisites
ROS2 Installation

Ensure you have ROS2 Jazzy installed:
# Verify ROS2 installation
ros2 --version
echo $ROS_DISTRO


MoveIt2 Installation

sudo apt update
sudo apt install ros-jazzy-moveit

# Verify MoveIt2 installation
ros2 pkg list | grep moveit


Workspace Setup
1. Create ROS2 Workspace

mkdir -p ~/scare_ws/src
cd ~/scare_ws/src


2. Clone This Repository

cd ~/scare_ws/src
git clone https://github.com/ariegweomamerie/scara_robot_ros2_moveit.git


MoveIt Setup Assistant Guide 🛠️

Step 1: Launch MoveIt Setup Assistant

ros2 launch moveit_setup_assistant setup_assistant.launch.py

Step 2: Create New MoveIt Configuration

    Click "Create New MoveIt Configuration Package"

    Click "Browse" and navigate to your URDF file

    Click "Load Files"

Step 3: Configure Self-Collision Matrix

    Go to "Self-Collisions" tab

    Click "Generate Collision Matrix"

    Adjust sampling density if needed

Step 4: Define Planning Groups
Arm Planning Group:

    Click "Add Group"

    Set Group Name: scara_arm

    Choose Kinematic Solver: kdl_kinematics_plugin/KDLKinematicsPlugin

    Add Joints for your SCARA robot

Step 5: Configure Robot Poses (Optional)

    Go to "Robot Poses" tab

    Click "Add Pose"

    Create useful poses like "home", "ready", etc.

Step 6: Set End Effector

    Go to "End Effectors" tab

    Click "Add End Effector"

    Set Name and select parent link

Step 7: Generate Configuration Files

    Go to "Configuration Files" tab

    Set output path for your MoveIt config package

    Click "Generate Package"


1. Start Screen

Initial screen when you load your URDF.

Here you choose the URDF file you want to generate a MoveIt config for.

Also lets you select the package path for the new MoveIt config package.

Action: Pick your urdf_to_sdf_gazebo.urdf and set the destination folder (scara_robot_moveit_config).

2. Self-Collision

Defines which robot links should or should not be checked for collisions.

MoveIt can ignore collisions for adjacent links or links that are physically constrained.

Action: Usually, just keep the default collision matrix initially; you can edit later manually.

3. Virtual Joints

Allows you to attach the robot to the world frame virtually.

Important for mobile bases or when the robot is fixed in a specific frame.

Action: For a stationary SCARA robot, attach a world frame at base_link.

4. Planning Groups

Groups of joints and links that MoveIt will plan for.

Examples: arm, gripper.

Action: Define your SCARA robot’s arm as a group, add the joints in the correct order.

5. Robot Poses

Predefined named robot poses you can load in RViz.

Useful for simulations, demos, or starting positions for planning.

Action: Optional, you can add home, ready, or pick poses.

6. End Effectors

Defines the robot’s tool or gripper.

Connects the end effector to a planning group.

Action: Add your gripper link if your SCARA robot has one.

7. Passive Joints

Joints that are not actuated or controlled.

MoveIt will ignore them for planning but still consider them in collisions.

Action: Optional if SCARA has any passive joints.

8. ROS 2 Control URDF Model

Loads the ros2_control compatible URDF (with transmission tags).

MoveIt uses this for simulation and controller integration.

Action: Use this if you want to integrate with ros2_control.

9. ROS 2 Controllers

Defines control interfaces for MoveIt to command joints.

Examples: JointTrajectoryController, PositionJointInterface.

Action: Needed for real robots or Gazebo simulations.

10. MoveIt Controllers

Maps planning groups to ROS 2 controllers.

Ensures that the planner knows which controller drives which group.

Action: Required if you plan to execute motions.

11. Perception

Configuration for sensors and scene perception.

Usually includes point clouds, depth cameras, or fake sensors in simulation.

Action: Optional unless you are adding perception pipelines.

12. Launch Files

Automatically generates launch files to run MoveIt in RViz, start move_group, or connect controllers.

Action: You’ll use these to launch MoveIt (moveit_rviz.launch.py etc.).

13. Author Information

Metadata for the generated package (author, email, license).

Action: Fill in your info; used in package.xml.

14. Configuration Files

Shows all generated YAML, SRDF, RViz configs.

Action: Check that all required files exist (kinematics.yaml, joint_limits.yaml, ompl_planning.yaml).




Building the Workspace

cd ~/scare_ws
colcon build
source install/setup.bash

Running the Demo
Launch MoveIt Demo

cd ~/scare_ws
source install/setup.bash
ros2 launch scara_robot_moveit_config demo.launch.py


Moving Your Robot with MoveIt 🎯
Method 1: Interactive Markers in RViz

    Launch the demo (as shown above)

    In RViz, you'll see:

        Orange robot: Start state

        Green robot: Goal state

        Interactive markers (3D arrows) around the end effector

    To plan a motion:

        Drag the interactive markers to position the end effector

        Click "Plan" in the MotionPlanning panel to see the planned path

        Click "Execute" to move the robot

Method 2: Using the MotionPlanning Panel

    In the MotionPlanning panel:

        Select planning group: scara_arm

        Set planning time (default: 5 seconds)

        Use "Query Goal State" to set target positions

    Plan and Execute:

        Plan: Generates a trajectory without moving

        Execute: Runs the trajectory on the robot

        Plan & Execute: Does both automatically

Project Structure
scare_ws/
├── src/
│   ├── scara_robot_cpp_pkg/
│   │   ├── urdf/                 # Robot URDF files
│   │   ├── meshes/               # 3D model meshes
│   │   ├── launch/               # Launch files
│   │   └── package.xml
│   └── scara_robot_moveit_config/
│       ├── config/               # MoveIt configuration
│       ├── launch/               # MoveIt launch files
│       └── package.xml
├── build/
├── install/
└── log/


📞 Support

For issues and questions:

    email me 

    Open an Issue on GitHub

🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for improvements.