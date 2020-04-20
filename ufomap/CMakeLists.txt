cmake_minimum_required(VERSION 3.8.2)
project(ufomap)

set(CMAKE_CXX_STANDARD 17)

# COMPILER SETTINGS (default: Release)
# use "-DCMAKE_BUILD_TYPE=Debug" in cmake for a Debug-build
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

MESSAGE ("\n")
MESSAGE (STATUS "${PROJECT_NAME} building as ${CMAKE_BUILD_TYPE}")

# COMPILER FLAGS
IF (CMAKE_COMPILER_IS_GNUCC)
  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error -march=native")
  SET (CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -DNDEBUG")
  SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
  # Shared object compilation under 64bit (vtable)
  ADD_DEFINITIONS(-fPIC)
ENDIF()


# Set full rpath http://www.paraview.org/Wiki/CMake_RPATH_handling
# (good to have and required with ROS)
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")

set(UFOMAP_MAJOR_VERSION 1)
set(UFOMAP_MINOR_VERSION 0)
set(UFOMAP_PATCH_VERSION 0)
set(UFOMAP_VERSION ${UFOMAP_MAJOR_VERSION}.${UFOMAP_MINOR_VERSION}.${UFOMAP_PATCH_VERSION})
set(UFOMAP_SOVERSION ${UFOMAP_MAJOR_VERSION}.${UFOMAP_MINOR_VERSION})

##### Doxygen
# First we can indicate the documentation build as an option and set it to ON by default
option(BUILD_DOC "Build documentation" ON)

# Check if Doxygen is installed
# find_package(Doxygen)
# if (DOXYGEN_FOUND)
#     # set input and output files
#     set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)
#     set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

#     # Request to configure the file
#     configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#     message("Doxygen build started")

#     # Note the option ALL which allows to build the docs together with the application
#     add_custom_target( doc_doxygen ALL
#         COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#         COMMENT "Generating API documentation with Doxygen"
#         VERBATIM )
# else (DOXYGEN_FOUND)
#   message("Doxygen need to be installed to generate the doxygen documentation")
# endif (DOXYGEN_FOUND)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${UFOMAP_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenMP_LIBRARIES}
  DEPENDS OpenMP LZ4
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
# find_library(LZ4 lz4)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UFOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/geometry/collision_checks.cpp
  src/math/pose6.cpp
  src/math/quaternion.cpp
  src/octree_rgb.cpp
  src/octree.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_LIBRARIES}
  ${LZ4_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)