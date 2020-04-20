cmake_minimum_required(VERSION 3.8.2)
project(ufomap_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  ufomap
)

add_message_files(
  FILES
  AABB.msg
  BoundingVolume.msg
#  Capsule.msg
#  Cone.msg
#  Cylinder.msg
#  Ellipsoid.msg
  Frustum.msg
  LineSegment.msg
  OBB.msg
  Plane.msg
  Point.msg
  Ray.msg
  Sphere.msg
#  Triangle.msg
  Ufomap.msg
  UfomapMetaData.msg
  UfomapWithPose.msg
)

add_service_files(
  FILES
  GetUfomap.srv
  BoundingVolumeQuery.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS geometry_msgs message_runtime std_msgs
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)