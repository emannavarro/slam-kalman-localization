#!/usr/bin/env python 

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla')
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')  # Define slam argument

    # Webots launcher configuration
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Robot description and driver
    robot_description_path = os.path.join(package_dir, 'resource', 'tesla_webots.urdf')
    tesla_driver = WebotsController(
        robot_name='vehicle',
        parameters=[{'robot_description': robot_description_path}],
        respawn=True
    )

    # Lane follower node
    lane_follower = Node(
        package='webots_ros2_tesla',
        executable='lane_follower',
    )

    # Lidar publisher node
    lidar_publisher = Node(
        package='webots_ros2_tesla',
        executable='lidar_publisher',
        name='lidar_publisher',
        output='screen'
    )

    # PointCloud2 to LaserScan conversion node
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/lidar/points/point_cloud'),  # Input topic from Lidar
            ('scan', '/scan')                           # Output LaserScan topic for SLAM
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'range_min': 0.1,
            'range_max': 250.0,
            'scan_time': 0.1
        }]
    )

    # SLAM Toolbox node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(package_dir, 'config', 'slam_toolbox_config.yaml')],
        condition=launch.conditions.IfCondition(slam),
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Static transform publisher (base_link to lidar)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
        name='static_transform_publisher'
    )

    # Define the launch description and conditional nodes
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='tesla_world.wbt',
            description='Choose one of the world files from `/webots_ros2_tesla/worlds` directory'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',  # Set slam to true by default
            description='Whether to launch SLAM Toolbox'
        ),
        webots,
        webots._supervisor,
        tesla_driver,
        lane_follower,
        lidar_publisher,
        pointcloud_to_laserscan,
        slam_toolbox,  # Only launches if slam:=true
        static_transform_publisher,  # Add the static transform publisher here

        # Shutdown action on Webots exit
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
