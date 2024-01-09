# Axebot Simulation

![Axebot in gazebo](doc/images/axebot.png)

The Axebot is an omnidirectional robot with three wheels that was initially developed by students at Universidade Federal da Bahia for robot soccer competitions. However, it was later used by the LaR UFBA for academic research in mobile robotics and control. This repository provides a simulation version of the robot to make it easier for researchers to validate algorithms.

## Table of Contents

- [Axebot Simulation](#axebot-simulation)
  - [Installation Premises](#installation-premises)
  - [Installing](#installing)
    - [Dependencies](#dependencies)
    - [Installing from source](#installing-from-source)
  - [Building](#building)
  - [Static code analysis](#static-code-analysis)
  - [Usage](#usage)
    - [Move to point](#move-to-point)
  - [Config files](#config-files)
  - [Nodes](#nodes)
  - [Contributing](#contributing)
  - [Developed Researches](#developed-researches)

## Installation Premises

1. This repository has been tested on [ROS2 Version] with [Classic Gazebo 11];

2. These instructions assume that you have already installed ROS2 Humble Hawskbill on your machine. If not, please follow the recommended [recommended ubuntu installation tutorial];

3. Before installing the package, you will need to have an ament workspace set up. If you don't have one, follow the instructions in the [Creating a workspace tutorial]. Once you have created the workspace, clone this repository in the source folder of your workspace.

## Installing

### Non ROS Dependencies

- [vcstool](https://github.com/dirk-thomas/vcstool): Refer to its repository for instructions on how to install.

### Installing from source

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```
cd ~/ros_ws/src
git clone https://github.com/mateusmenezes95/axebot.git
```

Install the binary dependencies by running the following command in the root of your workspace:

```
cd ~/ros_ws
rosdep init
rosdep update
sudo apt update
rosdep install --from-paths src/axebot --ignore-src -r -y --rosdistro humble
```

If all dependencies are already installed, you should see the message "All required rosdeps installed successfully."

Now, install the omnidirectional controller packager running:

```
cd ~/ros_ws/src
vcs import < axebot/axebot.humble.repos
```

## Building

Run the following command to build the package:

```
cd ~/ros_ws
colcon build --symlink-install --event-handlers console_direct+
```

> Run `colcon build --help` to understand the arguments passed!

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

> See [Source the overlay] to learn about underlay and overlay concepts.

## Static code analysis

Run the static code analysis with

> TODO

## Usage

To launch the Gazebo simulation and spawn the robot into it, run the following command:

```
ros2 launch axebot_gazebo axebot.launch.py
```

To move the robot in a straight line with a velocity of 0.5 m/s in the x-axis, execute the following command:

```
ros2 topic pub \
omnidirectional_controller/cmd_vel_unstamped \
-r 10 \
geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

> Alternatively, you can also send twist velocity commands through the rqt publisher plugin by running the command `ros2 run rqt_publisher rqt_publisher` and then configuring the command in the GUI.

### Move to point

To navigate the robot to a specific point (x, y) in 2D cartesian space, there is a simple move-to-point node that can be launched using the following command:

```
ros2 launch axebot_control move_axebot_to_point.launch.py
```

In a new terminal, you can command the robot to move to point (2, 2) by executing:

```
ros2 topic pub /go_to_goal/goal --once geometry_msgs/msg/Vector3 "{x: 2.0, y: 2.0, z: 0.0}"
```

> As an alternative, you can use the rqt_publisher to set the 2D points and interact with the Axebot.

This move-to-point node is a great starting point for those looking to develop high-level controllers for navigating the robot in an environment.

## Config files

- **[omnidirectional_controller.yaml](axebot_control/config/omnidirectional_controller.yaml):** Parameters of the omnidirectional controller.

## Nodes

When you launch the main file ([axebot.launch.py](axebot_gazebo/launch/axebot.launch.py)), the following nodes are initiated:

- `/controller_manager`
- `/gazebo`
- `/gazebo_ground_truth/gazebo_ros_p3d`
- `/gazebo_ros2_control`
- `/joint_state_broadcaster`
- `/omnidirectional_controller`
- `/robot_state_publisher`

Additionally, the following topics will also be available:

- `/clock [rosgraph_msgs/msg/Clock]`: This topic provides information about the current system time, it is used to keep the time-stamp of the data published in other topics consistent.
- `/dynamic_joint_states [control_msgs/msg/DynamicJointState]`: This topic provides the current states of the joints on a robot, such as position and velocity.
- `/gazebo_ground_truth/odom [nav_msgs/msg/Odometry]`: This topic provides odometry data from the Gazebo simulator, which includes information about the robot's position and orientation in the simulated environment.
- `/joint_states [sensor_msgs/msg/JointState]`: This topic provides the current states of the joints on a robot, such as position and velocity.
- `/omnidirectional_controller/cmd_vel_unstamped [geometry_msgs/msg/Twist]`: This topic provides the command velocity for an omnidirectional robot, which is a robot that can move in any direction.
- `/omnidirectional_controller/odom [nav_msgs/msg/Odometry]`: This topic provides odometry data for an omnidirectional robot, which includes information about the robot's position and orientation in the environment.
- `/parameter_events [rcl_interfaces/msg/ParameterEvent]`: This topic provides information about changes in the ROS 2 parameter server, which is a central repository for configuration information.
- `/performance_metrics [gazebo_msgs/msg/PerformanceMetrics]`: This topic provides performance metrics for the system, such as CPU usage, memory usage, and network traffic.
- `/robot_description [std_msgs/msg/String]`: This topic provides information about the robot, such as its physical properties, sensors, and actuators.
- `/rosout [rcl_interfaces/msg/Log]`: This topic provides log messages and diagnostic information from ROS 2 nodes.
- `/tf [tf2_msgs/msg/TFMessage]`: This topic provides information about the relationship between different coordinate frames in the system, such as the location of the robot's sensors in relation to the robot's base frame.
- `/tf_static [tf2_msgs/msg/TFMessage]`: This topic provides information about the relationship between different coordinate frames in the system, but for the static frames. It does not change over time.

> Topics description wrote by [ChatGPT]!!!

For more information about nodes, topics, services, and other elements of the Axebot simulation, refer to the [CLI Tools Tutorial].

## Contributing

To contribute to this package, you can either [open an issue](https://github.com/mateusmenezes95/axebot/issues) describing the desired subject or develop the feature yourself and  [submit a pull request](https://github.com/mateusmenezes95/axebot/pulls) to the main branch (in this case, humble).

If you choose to develop the feature yourself, please adhere to the [ROS 2 Code style and language] guidelines to improve code readability and maintainability.

## Developed Researches

M. S. Meneses, B. S. S. Pereira and T. L. M. Santos, "Robust reference tracking control subject to time-varying delay with future reference anticipation," 2023 Latin American Robotics Symposium (LARS), 2023 Brazilian Symposium on Robotics (SBR), and 2023 Workshop on Robotics in Education (WRE), Salvador, Brazil, 2023, pp. 367-372, doi: 10.1109/LARS/SBR/WRE59448.2023.10333024. [[link]](https://ieeexplore.ieee.org/document/10333024)

J. Santos, A. Conceição, T. Santos, and H. Ara ujo, “Remote control of an omnidirectional mobile robot with time-varying delay and noise attenuation,” *Mechatronics*, vol. 52, pp. 7–21, 2018, ISSN: 0957-4158. DOI: 10.1016/j.mechatronics.2018.04.003. [Online](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300606).

J. Santos, A. G. Conceição, and T. L. Santos, “Trajectory tracking of omni-directional mobile robots via predictive control plus a filtered smith predictor,” IFAC- PapersOnLine, vol. 50, no. 1, pp. 10 250–10 255, 2017, 20th IFAC World Congress, ISSN: 2405-8963. DOI: 10.1016/j.ifacol.2017.08.1286. [Online](https://www.sciencedirect.com/science/article/pii/S2405896317318013).

J. C. L. Barreto S., A. G. S. Conceição, C. E. T. Dórea, L. Martinez and E. R. de Pieri, "Design and Implementation of Model-Predictive Control With Friction Compensation on an Omnidirectional Mobile Robot," in *IEEE/ASME Transactions on Mechatronics*, vol. 19, no. 2, pp. 467-476, April 2014, doi: 10.1109/TMECH.2013.2243161. [[link]](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300606)

A. G. S. Conceicao, M. D. Correia, and L. Martinez, “Modeling and friction estimation for wheeled omnidirectional mobile robots,” *Robotica*, vol. 34, no. 9, pp. 2140–2150, 2016. [[link]](https://www.cambridge.org/core/journals/robotica/article/abs/modeling-and-friction-estimation-for-wheeled-omnidirectional-mobile-robots/85796F5CF60310022D35CB792EF80254)

[Creating a workspace tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
[recommended ubuntu installation tutorial]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
[Source the overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay
[ROS 2 Code style and language]: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#code-style-and-language-versions
[ROS2 Version]: https://docs.ros.org/en/humble/index.html
[Classic Gazebo 11]: https://classic.gazebosim.org/
[ChatGPT]: https://openai.com/blog/chatgpt/
[CLI Tools Tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html#
