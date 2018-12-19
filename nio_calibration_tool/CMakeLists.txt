cmake_minimum_required(VERSION 2.8.3)
project(nio_calibration_tool)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs 
	tf
	pcl_conversions
	pcl_ros
)

find_package(PCL 1.7 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
)


###########H
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include ${catkin_INCLUDE_DIRS}
        include/
)

add_executable(calib_board_detection_node src/CalibBoardDetectionNode.cpp src/CalibBoardDetector.cpp)
target_link_libraries(calib_board_detection_node ${catkin_LIBRARIES})
