# avoidsim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the TurtleBot3 simulation launch file
    # turtlebot3_gazebo_launch_file_dir = '/opt/ros/humble/share/turtlebot3_gazebo'
    turtlebot3_simulation_launch_file = '/home/gideon/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/maze3x3.launch.py'

    return LaunchDescription([
        # Include the launch file that launches TurtleBot3 simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_simulation_launch_file),
            launch_arguments={'TURTLEBOT3_MODEL': 'burger'}.items(),
        ),

        # Launch your executable
        Node(
            package='turtlebot3_obstacle_avoid',
            executable='turtlebot3_driver',
            name='turtlebot3_driver',
            output='screen',
        ),
    ])


