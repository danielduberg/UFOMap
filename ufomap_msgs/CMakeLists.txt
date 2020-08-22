cmake_minimum_required(VERSION 3.8.2)
project(ufomap_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ufomap REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ufomap.msg"
  "msg/UfomapWithPose.msg"
  "srv/GetUfomap.srv"
  "srv/BoundingBoxQuery.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
