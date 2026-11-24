# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Building-like 3D model of the SLAM map (occupied cells extruded to rack height).

    ros2 launch wsl_exploration extrude.launch.py [height:=2.4] [export_path:=~/warehouse_3d.obj]

Needs /rtabmap/map. In RViz: Add > By topic > /exploration/extruded_map > Marker.
Save the model: ros2 service call /map_extruder/export std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('height', default_value='2.4',
                              description='Extrusion height in metres (rack height).'),
        DeclareLaunchArgument('export_path', default_value='~/warehouse_3d.obj'),
        Node(package='wsl_exploration', executable='map_extruder', name='map_extruder',
             output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'height': LaunchConfiguration('height'),
                          'export_path': LaunchConfiguration('export_path')}]),
    ])
