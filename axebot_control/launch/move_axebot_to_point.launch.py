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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable

def generate_launch_description():
    spawn_gazebo_simulation = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' launch ',
            ' axebot_gazebo ',
            ' axebot.launch.py',
        ]],
        shell=True,
        output='screen'
    )

    move_to_point_node = Node(
        package="axebot_control",
        executable="go_to_goal",
    )

    move_to_point_node = Node(
        package="axebot_control",
        executable="go_to_goal",
    )

    rqt_publisher_node = Node(
        package="rqt_publisher",
        executable="rqt_publisher",
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
    )

    move_to_point_node_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_gazebo_simulation,
            on_start=[move_to_point_node]
        )
    )

    return LaunchDescription([
        spawn_gazebo_simulation,
        move_to_point_node_handler,
        rqt_publisher_node,
        plotjuggler_node
    ])
