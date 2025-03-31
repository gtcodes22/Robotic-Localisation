import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the URDF file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables for the full 6-DOF pose
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')  # Added z
    roll_pose = LaunchConfiguration('roll_pose', default='0.0')  # Added roll
    pitch_pose = LaunchConfiguration('pitch_pose', default='0.0')  # Added pitch
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')  # Added yaw

    # Declare launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify the x position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify the y position of the robot')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='Specify the z position of the robot')

    declare_roll_position_cmd = DeclareLaunchArgument(
        'roll_pose', default_value='0.0',
        description='Specify the roll orientation (radians) of the robot')

    declare_pitch_position_cmd = DeclareLaunchArgument(
        'pitch_pose', default_value='0.0',
        description='Specify the pitch orientation (radians) of the robot')

    declare_yaw_position_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Specify the yaw orientation (radians) of the robot')

    # Spawn the TurtleBot3 in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll_pose,  # Added roll
            '-P', pitch_pose,  # Added pitch
            '-Y', yaw_pose  # Added yaw
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare all launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_roll_position_cmd)
    ld.add_action(declare_pitch_position_cmd)
    ld.add_action(declare_yaw_position_cmd)

    # Add the spawning command
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
