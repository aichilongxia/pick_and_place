execute_process(COMMAND "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/arbotix_ros-indigo-devel/arbotix_sensors/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/arbotix_ros-indigo-devel/arbotix_sensors/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
