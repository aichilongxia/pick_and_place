# Install script for directory: /home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/lh/Moveit_control/turtlebot_arm-test/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/action" TYPE FILE FILES
    "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/action/BlockDetection.action"
    "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/action/InteractiveBlockManipulation.action"
    "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/action/PickAndPlace.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/msg" TYPE FILE FILES
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionAction.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionActionGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionActionResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionActionFeedback.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/BlockDetectionFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/msg" TYPE FILE FILES
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationAction.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationActionGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationActionResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationActionFeedback.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/InteractiveBlockManipulationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/msg" TYPE FILE FILES
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceAction.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceActionGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceActionResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceActionFeedback.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceGoal.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceResult.msg"
    "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/turtlebot_arm_block_manipulation/msg/PickAndPlaceFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/cmake" TYPE FILE FILES "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/turtlebot_arm_block_manipulation/catkin_generated/installspace/turtlebot_arm_block_manipulation-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/include/turtlebot_arm_block_manipulation")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/share/common-lisp/ros/turtlebot_arm_block_manipulation")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/python2.7/dist-packages/turtlebot_arm_block_manipulation")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/python2.7/dist-packages/turtlebot_arm_block_manipulation")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/turtlebot_arm_block_manipulation/catkin_generated/installspace/turtlebot_arm_block_manipulation.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/cmake" TYPE FILE FILES "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/turtlebot_arm_block_manipulation/catkin_generated/installspace/turtlebot_arm_block_manipulation-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation/cmake" TYPE FILE FILES
    "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/turtlebot_arm_block_manipulation/catkin_generated/installspace/turtlebot_arm_block_manipulationConfig.cmake"
    "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/turtlebot_arm_block_manipulation/catkin_generated/installspace/turtlebot_arm_block_manipulationConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation" TYPE FILE FILES "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation" TYPE EXECUTABLE FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/turtlebot_arm_block_manipulation/block_detection_action_server")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_detection_action_server")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation" TYPE EXECUTABLE FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/interactive_manipulation_action_server")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation" TYPE EXECUTABLE FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/pick_and_place_action_server")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation" TYPE EXECUTABLE FILES "/home/lh/Moveit_control/turtlebot_arm-test/devel/lib/turtlebot_arm_block_manipulation/block_manipulation_demo")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot_arm_block_manipulation/block_manipulation_demo")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation" TYPE DIRECTORY FILES "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/demo")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_arm_block_manipulation" TYPE DIRECTORY FILES "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/turtlebot_arm_block_manipulation/launch")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

