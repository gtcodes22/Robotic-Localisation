# avoidreal.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the TurtleBot3 bringup launch file for real robot
    turtlebot3_bringup_launch_file_dir = '/opt/ros/humble/share/turtlebot3_bringup'
    turtlebot3_bringup_launch_file = turtlebot3_bringup_launch_file_dir + '/launch/robot.launch.py'

    return LaunchDescription([
        # Include the launch file that brings up the real TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_bringup_launch_file),
            launch_arguments={
                'TURTLEBOT3_MODEL': 'burger',
                'use_sim_time': 'false'  # Important for real robot
            }.items(),
        ),

        # Launch your obstacle avoidance node
        Node(
            package='turtlebot3_obstacle_avoid',
            executable='turtlebot3_driver',
            name='turtlebot3_driver',
            output='screen',
            parameters=[{
                'use_sim_time': False  # Important for real robot
            }]
        ),
    ])