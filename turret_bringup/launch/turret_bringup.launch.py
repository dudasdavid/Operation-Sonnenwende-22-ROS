import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_turret_bringup = get_package_share_directory('turret_bringup')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turret_bringup, 'launch', 'realsense_bringup.launch.py'),
        )
    )

    turret_controller = Node(
            package='turret_control_py',
            executable='turret_controller',
            name='turret_controller',
            output='screen'
        )
    
    gun_controller = Node(
            package='turret_control_py',
            executable='gun_controller',
            name='turret_controller',
            output='screen'
        )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(realsense_launch)
    launchDescriptionObject.add_action(turret_controller)
    launchDescriptionObject.add_action(gun_controller)

    return launchDescriptionObject
