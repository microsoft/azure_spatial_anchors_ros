cmake_minimum_required(VERSION 2.8.3)
project(asa_ros)

# Deprecation Warning
message(DEPRECATION "The anchor_id member of FindAnchor.srv is deprecated and will be \
removed in a future release. Use the anchor_ids vector member instead.")

add_definitions(-std=c++1z)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cmake_policy(SET CMP0053 OLD)
include(cmake/asa_ros_dependencies_extras.cmake.in)

# Find package dependencies.
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)

include_directories(
  ${Gflags_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

# Find the full version of the spatial anchor library, obtained from debian
# packages (see github repo for more information on how to obtain these).
find_package(AzureSpatialAnchors REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/asa_interface.cpp
  src/asa_ros_provider.cpp
  src/asa_bag_reader.cpp
  src/internal/asa_helper.cpp
)
target_link_libraries(${PROJECT_NAME}
    AzureSpatialAnchors::AzureSpatialAnchors
    Eigen3::Eigen
    ${GLOG_LIBRARIES}
    ${Gflags_LIBRARIES}
)

############
# BINARIES #
############
cs_add_executable(asa_bag_reader
  src/asa_bag_reader_main.cpp
)
target_link_libraries(asa_bag_reader ${PROJECT_NAME})

cs_add_executable(asa_ros_node
  src/asa_ros_node.cpp
)
target_link_libraries(asa_ros_node ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()

cs_export(CFG_EXTRAS asa_ros_dependencies_extras.cmake
          INCLUDE_DIRS ${Gflags_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
          LIBRARIES ${Gflags_LIBRARIES} ${GLOG_LIBRARIES}
)
