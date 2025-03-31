#!/bin/bash

# Terminal 1 - Runs TurtleBot3 workspace setup, build, and launches 3x3 maze in Gazebo
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
colcon build --symlink-install --event-handlers console_direct+ && \
source ~/turtlebot3_ws/install/setup.bash && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_gazebo maze3x3.launch.py &&\
echo 'TurtleBot3 environment set up!' && exec bash"

# Terminal 2 - Runs Cartographer
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True; \\
exec bash"

# Terminal 3 - Open Terminal (will use later for mapping)
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
export TURTLEBOT3_MODEL=burger && \
echo 'Run this command later to save the map: 
mkdir -p ~/turtlebot3_ws/src/maps/sim/map_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/src/maps/sim/map_$(date +%Y%m%d_%H%M%S)/map' && exec bash"

# Terminal 4 - Activate the 'joy' node for Joystick Teleop
gnome-terminal -- bash -c "ls /dev/input/ && \
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js1 && \
exec bash"  #use 'ls /dev/input/' to confirm if joystick is js1. If only js0 and js1 are showing, then your joystick is js1.

# Terminal 5 - Teleop with PS4 Dualshock Joystick
gnome-terminal -- bash -c "~/ros2_ws/scripts/ps4_teleop.sh; exec bash"
