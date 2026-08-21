cmake_minimum_required(VERSION 3.8)
project(my_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install directories so ros2 can find them
install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install your teleop Python script
install(PROGRAMS
  src/race_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
