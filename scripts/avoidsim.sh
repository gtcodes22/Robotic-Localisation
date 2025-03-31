#!/bin/bash

bash -c "cd ~/turtlebot3_ws && \
colcon build --symlink-install --event-handlers console_direct+ && \
source ~/turtlebot3_ws/install/setup.bash && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_obstacle_avoid avoidsim.launch.py && \
exec bash"