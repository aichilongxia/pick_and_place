cmake_minimum_required(VERSION 2.8.3)
project(object_localization)

find_package(catkin REQUIRED COMPONENTS
             std_msgs
             sensor_msgs
             cmake_modules
             roscpp
             pcl_ros
             pcl_conversions
             tf 
             visualization_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED system filesystem)
include_directories(include)
include_directories(system
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )
catkin_package()
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_executable(object_localization src/object_localization.cpp
                                   src/point_cloud_preprocess.cpp
                                   src/geometry_model.cpp)
target_link_libraries(object_localization ${catkin_LIBRARIES})

