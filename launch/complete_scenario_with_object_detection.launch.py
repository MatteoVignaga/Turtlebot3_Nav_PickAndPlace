#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Hye-jong KIM
# Adapted by: Matteo Vignaga

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    this_pkg_share = get_package_share_directory('template_package')

    # MoveIt
    moveit_launch_dir = os.path.join(
        this_pkg_share, 
        'launch'
    )

    # move_group
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([moveit_launch_dir, '/move_group.launch.py']),
            launch_arguments={
                'use_sim': 'true',
            }.items()
        )
    ld.add_action(move_group_launch)

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # Gazebo 
    world_path = PathJoinSubstitution(
        [
            this_pkg_share,
            'gazebo',
            'worlds',
            'complete_scenario_with_obj_detection.world'
        ]
    )

    gazebo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    this_pkg_share,
                    'launch',
                    'gazebo.launch.py'
                ]
            )
        ),
        launch_arguments={
            'world': world_path,
            'x_pose': '-2.0',
            'y_pose': '-0.5',
            'z_pose': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items()
    )
    ld.add_action(gazebo_control_launch)

    # ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py 
    # map_yaml_file:=/home/matteo/template_ws/src/template_package/navigation/maps/spacious_map.yaml 
    # params_file:=/home/matteo/template_ws/src/template_package/navigation/params/turtlebot3.yaml 
    navigation_launchfile_path = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_navigation2'),
        'launch',
        'navigation2.launch.py'
    )
    map = os.path.join(
        this_pkg_share,
        'navigation',
        'maps',
        'complete_scenario_with_obj_detection_map.yaml'
    )
    params_file = os.path.join(
        this_pkg_share,
        'navigation',
        'params',
        'turtlebot3.yaml'
    )
    waffle_params_file = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_navigation2'),
        'param',
        'turtlebot3.yaml'
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launchfile_path),
        launch_arguments={
            'map_yaml_file': map,
            'params_file': waffle_params_file,
            'start_rviz': 'true'        # not sure why this is required despite it being default value
        }.items()
    )
    ld.add_action(navigation_launch)

    # ArUco detection
    aruco_launch_file = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'launch',
        'aruco_recognition.launch.py'
    )
    aruco_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_launch_file)
    )
    ld.add_action(aruco_recognition_launch)

    # requires transformations so wait
    """ custom_aruco_broadcaster = Node(
        package='template_package',
        executable='read_marker_position',
        output='screen',
    )
    ld.add_action(custom_aruco_broadcaster) """

    return ld
