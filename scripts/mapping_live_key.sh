#!/bin/bash

# Credits: Thanks to https://emanual.robotis.com/ for the original commands and GitHub repositories (https://github.com/ROBOTIS-GIT/turtlebot3)

# First ssh into the Turtlebot and then 'bringup' the robot within the Turtlebot's remote interface.

# SSH Instructions
# ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}

# Turtlebot's SSH Ubuntu Interface (aka TurtleBot3 SBC)
# export TURTLEBOT3_MODEL=burger
# ros2 launch turtlebot3_bringup robot.launch.py 
# ALWAYS ensure to 'bringup' the TurtleBot before conducting the following operations.

# Finally, ensure that you add the following to the .bashrc file (for both your PC and the TurtleBot3 SBC):

# export ROS_DOMAIN_ID=30 
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# then source ~/.bashrc

#--------------------------------------------

# Run these commands within your PC, NOT the TurtleBot3 SBC.
# This script is similar in some aspects to the mapping_sim_key.sh script.

# Terminal 1 - Runs Cartographer
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_cartographer cartographer.launch.py && \
exec bash" # Do not copy the command from the SLAM simulation tutorial 'as-is', because it has an extra condition "use_sim_time:=True", which will then launch RViz2 to map out a Gazebo environment instead of the real robot's environment.

# Terminal 2 - Open Terminal (will use later for mapping)
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger && \
echo 'Run this command later to save the map: 
mkdir -p ~/turtlebot3_ws/src/maps/live/map_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/src/maps/live/map_$(date +%Y%m%d_%H%M%S)/map' && exec bash"

# Terminal 3 - Activate the Teleop Keyboard
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger && \
ros2 run turtlebot3_teleop teleop_keyboard && \
exec bash"