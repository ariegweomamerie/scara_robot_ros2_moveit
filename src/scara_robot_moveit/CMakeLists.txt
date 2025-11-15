cmake_minimum_required(VERSION 3.8)
project(scara_robot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create executable - FIXED: Use consistent name
add_executable(scara_robot_moveit src/scara_robot_moveit.cpp)

target_include_directories(scara_robot_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scara_robot_moveit PUBLIC c_std_99 cxx_std_17)

# FIXED: Use the EXACT SAME target name as add_executable
ament_target_dependencies(
  scara_robot_moveit  # This MUST match the add_executable name
  rclcpp
  moveit_ros_move_group
  moveit_ros_planning_interface
  geometry_msgs
)

# FIXED: Install the same target name
install(TARGETS scara_robot_moveit
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()