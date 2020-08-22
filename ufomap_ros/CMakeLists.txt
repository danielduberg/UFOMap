cmake_minimum_required(VERSION 3.8.2)
project(ufomap_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ufomap REQUIRED)
find_package(ufomap_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  ufomap
  ufomap_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/conversions.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

if(TARGET test_sensor_msgs)
  rosidl_target_interfaces(${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()