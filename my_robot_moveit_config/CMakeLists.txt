cmake_minimum_required(VERSION 3.22)
project(my_robot_moveit_config)

find_package(ament_cmake REQUIRED)

# Install launch and config directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install the hidden setup assistant file
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()