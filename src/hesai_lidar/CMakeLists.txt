cmake_minimum_required(VERSION 3.5)
project(hesai_lidar)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED) # Assuming yaml-cpp-vendor for ROS2
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PandarPacket.msg"
    "msg/PandarScan.msg"
    DEPENDENCIES std_msgs sensor_msgs # Add other message dependencies if any
)

# C++ code will be added here in the next step
# add_library(PandarGeneral ...)
# add_library(PandarGeneralSDK ...)
# add_executable(hesai_lidar_node ...)
# add_library(cloud_nodelet ...)

ament_package()
