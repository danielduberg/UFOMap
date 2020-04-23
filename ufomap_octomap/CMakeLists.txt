cmake_minimum_required(VERSION 2.8.3)
project(ufomap_octomap)

set(CMAKE_CXX_STANDARD 17)

# find_package(catkin REQUIRED)
find_package(ufomap REQUIRED)
find_package(octomap REQUIRED)

# include_directories(
#   include
# )

add_library(${PROJECT_NAME} INTERFACE)

set(header_files
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/conversions.h
)

target_sources(${PROJECT_NAME} INTERFACE "$<BUILD_INTERFACE:${header_files}>")

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  # $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>>
)
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>)

include( CMakePackageConfigHelpers )
set(CONFIG_PACKAGE_INSTALL_DIR lib/cmake/${PROJECT_NAME})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        DESTINATION lib)

    install( EXPORT ${PROJECT_NAME}-targets
      DESTINATION
        ${CONFIG_PACKAGE_INSTALL_DIR}
    )

    install( FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-config.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
      DESTINATION
        ${CONFIG_PACKAGE_INSTALL_DIR} )

# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}_Targets
#   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# include(CMakePackageConfigHelpers)
# write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
#   VERSION ${PROJECT_VERSION}
#   COMPATIBILITY SameMajorVersion
# )

# configure_package_config_file(
#   "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
#   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#   INSTALL_DESTINATION
#   ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
# )

# install(EXPORT ${PROJECT_NAME}_Targets
#   FILE ${PROJECT_NAME}Targets.cmake
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
# )

# install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
# )

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/SI DESTINATION include)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#  )