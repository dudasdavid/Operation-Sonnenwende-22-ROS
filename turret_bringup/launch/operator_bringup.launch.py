import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    
    pkg_bringup = FindPackageShare('turret_bringup')
    pkg_description = FindPackageShare('turret_description')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    dummy_jsp_arg = DeclareLaunchArgument(
        'jsp', default_value='true',
        description='Open fake joint state publisher'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='operator_turret_view.rviz',
        description='RViz config file'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='turret_standalone.xacro',
        description='Name of the URDF description to load'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_description,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp'))
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bringup, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Relay node to republish /rgbd/color/camera_info to /rgbd/camera/image_raw/camera_info
    relay_color_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgbd_color_camera_info',
        output='screen',
        arguments=['rgbd/color/camera_info', 'rgbd/color/image_raw/camera_info']
    )

    relay_infra_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgbd_infra_camera_info',
        output='screen',
        arguments=['rgbd/infra1/camera_info', 'rgbd/infra1/image_rect_raw/camera_info']
    )
    
    relay_depth_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgbd_depth_camera_info',
        output='screen',
        arguments=['rgbd/aligned_depth_to_color/camera_info', 'rgbd/aligned_depth_to_color/image_raw/camera_info']
    )

    ballistic_marker = Node(
        package='turret_control_py',
        executable='ballistic_marker',
        name='nerf_trajectory',
        output='screen',
        parameters=[{
            'world_frame': 'turret_base',
            'link_name': 'gun_ee_link',
            'pan_joint_name': 'pan_joint',
            'tilt_joint_name': 'tilt_joint',
            'muzzle_speed': 9.0,
            'mass': 0.0016,
            'drag_coefficient': 1.1,
            'cross_section_area': 3.14e-4,
            'ground_z': -1.0,
            'muzzle_offset_xyz': [0.0, 0.0, 0.0],
            'barrel_axis': 'x',
        }]
    )
    
    aiming_marker = Node(
        package='turret_control_py',
        executable='aiming_marker',
        name='aiming_marker',
        parameters=[{
            'frame_id': 'turret_base',
            'topic': 'target_pose',
            'scale': 0.3,
            'initial_xyz': [1.0, 0.0, 0.0],
            'zero_orientation_on_publish': True,
        }]
    )

    auto_aim = Node(
            package='turret_control_py',
            executable='auto_aim',
            name='auto_aim',
            parameters=[{
                'target_pose_topic': '/target_pose'
            }]
        )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(dummy_jsp_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(jsp_gui_node)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(relay_color_camera_info)
    launchDescriptionObject.add_action(relay_infra_camera_info)
    launchDescriptionObject.add_action(relay_depth_camera_info)
    launchDescriptionObject.add_action(ballistic_marker)
    launchDescriptionObject.add_action(aiming_marker)
    launchDescriptionObject.add_action(auto_aim)

    return launchDescriptionObject
