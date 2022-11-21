# Copyright (c) 2022 Mateus Menezes

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.logging import LaunchLogger
import xacro

def generate_launch_description():
    pkg_name = 'axebot_description'

    only_wheel = LaunchConfiguration('only_wheel')

    only_wheel_arg = DeclareLaunchArgument(
        name='only_wheel',
        default_value='false',
        description='True if to view only one omni wheel, false otherwise.',
        choices=['true', 'false']
    )

    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'axebot.urdf.xacro'
    )

    rviz_robot_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'axebot.rviz'
    )

    rviz_only_wheel_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'wheel_only.rviz'
    )

    robot_description_urdf = xacro.process_file(
        xacro_file, mappings={'only_one_wheel' : 'false'}
    ).toxml()

    only_omni_wheel_urdf = xacro.process_file(
        xacro_file, mappings={'only_one_wheel' : 'true'}
    ).toxml()

    return LaunchDescription([
        only_wheel_arg,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=IfCondition(only_wheel),
            arguments=['-d', rviz_only_wheel_config]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=UnlessCondition(only_wheel),
            arguments=['-d', rviz_robot_config]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': only_omni_wheel_urdf}],
            condition=IfCondition(only_wheel)
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_urdf}],
            condition=UnlessCondition(only_wheel)
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    ])
