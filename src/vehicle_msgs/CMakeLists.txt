cmake_minimum_required(VERSION 3.5)
project(vehicle_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/TrackCone.msg"
  "msg/Track.msg"
  "msg/Waypoint.msg"
  "msg/WaypointsArray.msg"
  "msg/ObjectInfo.msg"
  "msg/AvoidAngleArray.msg"
  "msg/Car.msg"
  "msg/CarObject.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()