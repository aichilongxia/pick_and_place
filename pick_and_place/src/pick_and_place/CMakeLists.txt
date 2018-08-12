cmake_minimum_required(VERSION 2.8.3)
project(pick_and_place)

find_package(catkin REQUIRED
        COMPONENTS
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	cmake_modules
	geometric_shapes
        tf  
        roscpp 
        visualization_msgs 
        arbotix_msgs
        std_msgs              
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
        tf
        roscpp 
        visualization_msgs 
        arbotix_msgs
        std_msgs
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
#link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Eigen_INCLUDE_DIRS})
#include_directories(include
#                    SYSTEM
#                    ${Boost_INCLUDE_DIRS}
#                    ${catkin_INCLUDE_DIRS}
#                    ${Eigen_INCLUDE_DIRS}
#                   )

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_INCLUDE_DIRS})
