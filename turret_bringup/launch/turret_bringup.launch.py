import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    
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

    launchDescriptionObject.add_action(turret_controller)
    launchDescriptionObject.add_action(gun_controller)

    return launchDescriptionObject
