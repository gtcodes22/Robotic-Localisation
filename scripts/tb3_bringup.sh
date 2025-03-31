#!/bin/bash

gnome-terminal -- bash -c "
    export TURTLEBOT3_MODEL=burger && \
    ros2 launch turtlebot3_bringup robot.launch.py; \\
    exec bash"