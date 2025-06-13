cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gripper_srv REQUIRED)

# Interface generation (msgs + actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TaskStep.msg
  action/MoveToPose.action
  action/ExecuteTaskSequence.action
  DEPENDENCIES geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executable
add_executable(manipulator_control_node src/manipulator_control_node.cpp)
ament_target_dependencies(manipulator_control_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  gripper_srv
  builtin_interfaces
)

target_link_libraries(manipulator_control_node ${cpp_typesupport_target})

add_executable(task_sequence_client src/task_sequence_client.cpp)
ament_target_dependencies(task_sequence_client
  rclcpp
  rclcpp_action
  geometry_msgs
  builtin_interfaces
)
target_link_libraries(task_sequence_client ${cpp_typesupport_target})

# Install
install(TARGETS manipulator_control_node
  task_sequence_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing support (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
