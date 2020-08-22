cmake_minimum_required(VERSION 3.8.2)
project(ufomap_mapping)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(ufomap REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(ufomap_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  ufomap_msgs
  ufomap_ros
)

include_directories(
  include
)

add_library(${PROJECT_NAME}_server
  src/server.cpp
)

ament_target_dependencies(${PROJECT_NAME}_server
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_server
  ${OpenMP_LIBRARIES}
  ${LZ4_LIBRARIES}
)

add_executable(${PROJECT_NAME}_server_main 
  src/server_main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_server_main 
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_server_main
  ${PROJECT_NAME}_server
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}_server
  ${PROJECT_NAME}_server_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()