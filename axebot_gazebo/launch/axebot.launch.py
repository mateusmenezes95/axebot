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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():
    pkg_name = 'axebot_description'

    launch_rviz = LaunchConfiguration('launch_rviz')

    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='False',
        description='True if to launch rviz, false otherwise'
    )

    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'axebot.urdf.xacro'
    )

    rviz_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'axebot.rviz'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config]
    )

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}],
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'axebot'],
                        output='screen'
                        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnidirectional_controller"],
    )

    omni_base_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_base_controller_spawner]
        )
    )

    rviz_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=omni_base_controller_spawner,
            on_exit=[rviz_node]
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'),
                                       '/gazebo.launch.py']),
    )

    return LaunchDescription([
        launch_rviz_arg,
        joint_state_broadcaster_event_handler,
        omni_base_controller_event_handler,
        robot_state_publisher_node,
        spawn_entity,
        gazebo,
        rviz_event_handler
    ])
