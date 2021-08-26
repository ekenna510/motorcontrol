#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    #TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    #usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    motorcontrolconfig = LaunchConfiguration(
        'mc_config_dir',
        default=os.path.join(
            get_package_share_directory('motorcontrol'),
            'config',
            'motorcontrolbaselink.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'mc_config_dir',
            default_value=motorcontrolconfig,
            description='Full path to motorcontrol parameter file to load'),


        Node(
            package='motorcontrol',
            executable='motorcontrol',
            parameters=[motorcontrolconfig],
            arguments=[],
            #prefix=['gdb -ex "break main" -ex run  --args'],            
            output='screen'),
    ])

