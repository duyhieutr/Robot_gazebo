# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
RTAB-Map 3D LiDAR SLAM. Includable from wsl_sim or runnable against a rosbag.

    ros2 launch wsl_slam slam.launch.py [rgbd:=true]

rgbd:=true adds the RGB-D camera (/camera/image_raw, /camera/depth/image_raw,
/camera/camera_info) to the nodes RTAB-Map stores. Registration and the
occupancy grid still come from the LiDAR; the images give the 3D map its colour
and density (/rtabmap/cloud_map). Needs the robot started with rgbd:=true too.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory('wsl_slam')
    rgbd = LaunchConfiguration('rgbd').perform(context).lower() == 'true'

    # Later files override earlier ones.
    parameters = [os.path.join(share, 'config', 'rtabmap.yaml')]
    remappings = [('scan_cloud', '/lidar/points')]
    if rgbd:
        parameters.append(os.path.join(share, 'config', 'rtabmap_rgbd.yaml'))
        remappings += [
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
        ]
    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time'),
                       'database_path': LaunchConfiguration('database_path')})

    return [Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=remappings,
        # Start every run with an empty map; the previous database is replaced.
        arguments=['--delete_db_on_start'],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/wsl_rtabmap.db'),
            description='RTAB-Map database written during the run (map, graph, scans).'),
        DeclareLaunchArgument(
            'rgbd', default_value='false',
            description='Also subscribe to the RGB-D camera for a coloured 3D map.'),
        OpaqueFunction(function=launch_setup),
    ])
