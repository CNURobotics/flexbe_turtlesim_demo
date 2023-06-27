cmake_minimum_required(VERSION 3.5)
project(flexbe_turtlesim_demo_flexbe_states)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  tests
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
