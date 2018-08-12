#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/arbotix_ros/arbotix_sensors"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lh/Moveit_control/turtlebot_arm-test/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lh/Moveit_control/turtlebot_arm-test/install/lib/python2.7/dist-packages:/home/lh/Moveit_control/turtlebot_arm-test/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lh/Moveit_control/turtlebot_arm-test/build" \
    "/usr/bin/python" \
    "/home/lh/Moveit_control/turtlebot_arm-test/src/turtlebot_arm/arbotix_ros/arbotix_sensors/setup.py" \
    build --build-base "/home/lh/Moveit_control/turtlebot_arm-test/build/turtlebot_arm/arbotix_ros/arbotix_sensors" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/lh/Moveit_control/turtlebot_arm-test/install" --install-scripts="/home/lh/Moveit_control/turtlebot_arm-test/install/bin"
