cmake_minimum_required(VERSION 3.8)
project(move_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalWithHeading.msg"
  DEPENDENCIES builtin_interfaces
)

install(PROGRAMS move_pkg/nav2_ros_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_ros_controller
)

install(PROGRAMS move_pkg/virtual_robot_from_cmdvel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME virtual_robot_from_cmdvel
)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml TOPICS.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
