cmake_minimum_required(VERSION 3.8)
project(v2i_gridcalib_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RasterizePointCloud.srv"
)

ament_package()