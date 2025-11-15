from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    # Initialize arguments
    rviz_config = LaunchConfiguration("rviz_config")

    # Get moveit_config
    moveit_config = MoveItConfigsBuilder(
        "urdf_to_sdf_gazebo", package_name="scara_robot_moveit_config"
    ).to_moveit_configs()

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            # Explicitly set controller parameters
            {
                "moveit_simple_controller_manager": {
                    "controller_names": ["scara_arm_controller", "scara_gripper_controller"],
                    "scara_arm_controller": {
                        "type": "follow_joint_trajectory",
                        "action_ns": "scara_arm_controller",
                        "default": True,
                        "joints": [
                            "Revolute 10",
                            "Revolute 7", 
                            "Revolute 8",
                            "Slider 11",
                            "Slider 4",
                            "Slider 5"
                        ]
                    },
                    "scara_gripper_controller": {
                        "type": "follow_joint_trajectory", 
                        "action_ns": "scara_gripper_controller",
                        "default": False,
                        "joints": []  # Add gripper joints if any
                    }
                }
            }
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("scara_robot_moveit_config"), "config", rviz_config
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Controller Manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
                FindPackageShare("scara_robot_moveit_config"),
                "config",
                "ros2_controllers.yaml"
            ])
        ],
        output="both",
    )

    # Spawn Controllers
    spawn_controllers_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scara_arm_controller", "-c", "/controller_manager"],
        output="both",
    )

    spawn_joint_state_node = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="both",
    )

    nodes = [
        rsp_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
        static_tf_node,
        controller_manager_node,
        spawn_controllers_node,
        spawn_joint_state_node,
    ]

    return LaunchDescription(declared_arguments + nodes)