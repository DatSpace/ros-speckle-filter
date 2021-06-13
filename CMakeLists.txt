cmake_minimum_required(VERSION 3.0.2)
project(r1_laser_filter)

set(CMAKE_CXX_STANDARD 11)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters
  laser_geometry pluginlib angles dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/SpeckleFilter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_scan_filter
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
  )

##############################################################################
# Build
##############################################################################

add_library(laser_scan_filter
  src/laser_scan_filter.cpp
  src/speckle_filter.cpp
)
target_link_libraries(laser_scan_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(laser_scan_filter ${PROJECT_NAME}_gencfg)

##############################################################################
# Install
##############################################################################

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_filter_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
