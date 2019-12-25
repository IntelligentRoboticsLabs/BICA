# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_A_cmd = Node(
        package='bica_examples',
        node_executable='node_A',
        node_name='A',
        output='screen',
        parameters=[])

    node_B_cmd = Node(
        package='bica_examples',
        node_executable='node_B',
        node_name='B',
        output='screen',
        parameters=[])

    node_C_cmd = Node(
        package='bica_examples',
        node_executable='node_C',
        node_name='C',
        output='screen',
        parameters=[])

    node_D_cmd = Node(
        package='bica_examples',
        node_executable='node_D',
        node_name='D',
        output='screen',
        parameters=[])

    ld = LaunchDescription()

    ld.add_action(node_A_cmd)
    ld.add_action(node_B_cmd)
    ld.add_action(node_C_cmd)
    ld.add_action(node_D_cmd)

    return ld
