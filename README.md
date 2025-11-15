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
4. Planning Groups

# scara_robot_ros2_moveit

This repository contains a ROS 2 workspace for a SCARA robot. It includes the URDF model, MoveIt 2 configuration for motion planning, and launch files for simulation and visualization. The robot model is provided with STL meshes and configured with ros2_control.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
    - [ROS 2 installation](#ros-2-installation)
    - [MoveIt 2 installation](#moveit-2-installation)
- [Workspace setup](#workspace-setup)
- [MoveIt Setup Assistant Guide](#moveit-setup-assistant-guide)
- [Building the workspace](#building-the-workspace)
- [Running the demo](#running-the-demo)
- [Moving your robot with MoveIt](#moving-your-robot-with-moveit)
- [Project structure](#project-structure)
- [Troubleshooting & runtime checklist](#troubleshooting--runtime-checklist)
- [Contributing](#contributing)

## Overview

This project provides a ROS 2 implementation of a SCARA robot with MoveIt 2 integration, including:

- URDF robot model with 3D meshes
- MoveIt 2 configuration for motion planning
- ros2_control interfaces and controller configs
- RViz visualization configurations and launch files

## Prerequisites

### ROS 2 installation

Ensure you have ROS 2 (Jazzy or your target distro) installed. To verify:

```bash
ros2 --version
echo $ROS_DISTRO
```

### MoveIt 2 installation

Install MoveIt 2 for your ROS 2 distro. For Jazzy the package name is `ros-jazzy-moveit`:

```bash
sudo apt update
sudo apt install ros-jazzy-moveit
```

Verify installation:

```bash
ros2 pkg list | grep moveit
```

## Workspace setup

1. Create a workspace and clone this repository:

```bash
mkdir -p ~/scare_ws/src
cd ~/scare_ws/src
git clone <https://github.com/ariegweomamerie/scara_robot_ros2_moveit.git>
```

2. Build the workspace (see "Building the workspace" below).

## MoveIt Setup Assistant Guide

1. Launch the MoveIt Setup Assistant:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

2. Create a new MoveIt configuration package and point it to the URDF `urdf_to_sdf_gazebo.urdf`.
3. Configure self-collisions, virtual joints, planning groups, robot poses, end effector, passive joints, and controllers as needed.
4. Generate the configuration package into `scara_robot_moveit_config`.

Notes and common actions

- For a stationary SCARA robot, attach a virtual world joint at `base_link`.
- When defining planning groups, use consistent joint names that match your URDF and controller configs.

## Building the workspace

From the workspace root:

```bash
cd ~/scare_ws
colcon build
source install/setup.bash
```

For debug builds you can run:

```bash
colcon build --mixin debug
```

## Running the demo

After building and sourcing:

```bash
source install/setup.bash
ros2 launch scara_robot_moveit_config demo.launch.py
```

## Moving your robot with MoveIt

Method 1 — Interactive markers in RViz

- Launch the demo (above).
- Use interactive markers to set a goal pose and plan/execute from RViz.

Method 2 — MotionPlanning panel

- Select planning group `scara_arm`.
- Set planning time and request a plan.

This repository also contains a small planning-only node (does not execute trajectories) for testing planning behavior.

## Project structure

```
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
```

## Troubleshooting & runtime checklist

If MoveIt complains about missing joint states or execution aborts, follow these steps on the machine where you run the robot and controllers:

1. Verify joint states are being published:

```bash
ros2 topic list | grep joint_states
ros2 topic echo /joint_states -n1
```

2. If `/joint_states` is missing or has stale timestamps, spawn the joint state broadcaster and controllers using the helper script included in the workspace root:

```bash
cd ~/scare_ws
source install/setup.bash
./scripts/spawn_controllers.sh /controller_manager
```

3. Verify controllers are listed:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

4. If MoveIt warns about kinematics plugins, ensure the kinematics plugin package is installed (example for KDL solver on Jazzy):

```bash
sudo apt update
sudo apt install ros-jazzy-kdl-kinematics
```

5. Common fixes

- Joint names with spaces can be problematic; consider renaming joints to simple identifiers (for example `revolute_1`, `prismatic_z`) if you encounter mapping issues.
- Ensure timestamps on `joint_states` (header.stamp) are correct (not `0.0`) and that clocks are synced if running across machines.

## Contributing

Contributions are welcome! Please open issues or submit pull requests for improvements.
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

## Runtime checklist (quick)

If MoveIt complains about missing joint states or execution aborts, follow these steps on the machine where you run the robot and controllers:

1. Verify joint states are being published:

```bash
ros2 topic list | grep joint_states
ros2 topic echo /joint_states -n1
```

2. If `/joint_states` is missing or has stale timestamps, spawn the joint state broadcaster and controllers using the helper script included in the workspace root:

```bash
cd ~/scare_ws
source install/setup.bash
./scripts/spawn_controllers.sh /controller_manager
```

3. Verify controllers are listed:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

4. If MoveIt warns about kinematics plugins, ensure the kinematics plugin package is installed (for KDL solver):

```bash
sudo apt update
sudo apt install ros-jazzy-kdl-kinematics
```

If your ROS distro differs (not Jazzy), replace the package name accordingly.

5. Common fixes:
- Joint names with spaces can be problematic; consider renaming joints to simple identifiers (e.g. `revolute_1`, `prismatic_z`) if you encounter mapping issues between URDF and controllers.
- Ensure timestamps on joint_state messages are correct (not 0.0) and that clocks are synced if across machines.

If you'd like, I can (a) add controller spawn into a launch file, or (b) help rename joints for better tooling compatibility.