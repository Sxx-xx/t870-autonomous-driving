cmake_minimum_required(VERSION 3.5)
project(ublox_serialization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

ament_export_include_directories(include)

ament_package()

install(DIRECTORY include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE
)