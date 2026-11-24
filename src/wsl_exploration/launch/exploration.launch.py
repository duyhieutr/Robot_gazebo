# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Frontier exploration on top of a running SLAM + Nav2 system. Includable from wsl_sim.

    ros2 launch wsl_exploration exploration.launch.py [params_file:=...]

Needs /rtabmap/map and Nav2's navigate_to_pose action, i.e. warehouse.launch.py
with slam:=true nav:=true. Normally started with explore:=true instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('wsl_exploration'), 'config', 'exploration.yaml'),
            description='Parameters of the frontier_explorer node.'),
        Node(package='wsl_exploration', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
