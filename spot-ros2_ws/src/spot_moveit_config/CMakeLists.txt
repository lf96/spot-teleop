cmake_minimum_required(VERSION 3.22)
project(spot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(spot_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)

# Build servo_pose_tracking_spot executable
add_executable(servo_pose_tracking_spot src/servo_pose_tracking_spot.cpp)
ament_target_dependencies(servo_pose_tracking_spot
  rclcpp
  std_msgs
  geometry_msgs
  moveit_servo
  moveit_ros_planning
  tf2_ros
)

add_executable(ee_goal_executor_node src/ee_goal_executor_node.cpp)
ament_target_dependencies(ee_goal_executor_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  shape_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

install(TARGETS servo_pose_tracking_spot
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ee_goal_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(move_action_proxy src/move_action_proxy.cpp)
ament_target_dependencies(move_action_proxy
  rclcpp
  rclcpp_action
  moveit_msgs
)
install(TARGETS move_action_proxy
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/continuos_to_spot.py
  scripts/spot_pose_tracker.py
  scripts/target_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
