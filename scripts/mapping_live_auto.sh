#!/bin/bash

# Terminal 1 - Runs TurtleBot3 workspace setup, build, and launches 3x3 maze in Gazebo
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
colcon build --symlink-install --event-handlers console_direct+ && \
source ~/turtlebot3_ws/install/setup.bash && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_obstacle_avoid avoidreal.launch.py &&\
echo 'TurtleBot3 environment set up!' && exec bash"

# Terminal 2 - Runs Cartographer
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_cartographer cartographer.launch.py; \\
exec bash"

# Terminal 3 - Open Terminal (will use later for mapping)
gnome-terminal -- bash -c "cd ~/turtlebot3_ws && \
export TURTLEBOT3_MODEL=burger && \
echo 'Run this command later to save the map: 
mkdir -p ~/turtlebot3_ws/src/maps/live/map_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/src/maps/live/map_$(date +%Y%m%d_%H%M%S)/map' && exec bash"
