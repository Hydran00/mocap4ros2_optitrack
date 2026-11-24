# Copyright 2021 Institute for Robotics and Intelligent Machines,
#                Georgia Institute of Technology
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
#
# Author: Christian Llanes <christian.llanes@gatech.edu>
# Author: David Vargas Frutos <david.vargas@urjc.es>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = 'mocap4r2_optitrack_driver'
    config = PathJoinSubstitution([
        FindPackageShare(package), 'config', 'mocap4r2_optitrack_driver_params.yaml',
    ])
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('config_file', default_value=config),
        Node(
            package=package,
            executable='optitrack_driver',
            name='mocap4r2_optitrack_driver_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {'autostart': 'activate'}],
        ),
    ])
