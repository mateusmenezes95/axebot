# Axebot Simulation

![Axebot in gazebo](doc/images/axebot.png)

The Axebot is an omnidirectional robot with three wheels. The physical robot is actuated by three DC motors controlled by one embedded system.

Students of Universidade Federal da Bahia initially developed the Axebot for robot soccer competitions. However, afterward, the robot was used by the LaR UFBA to carry out academic research in several areas of mobile robotics and control. The section [Developed Research](#developed-research) lists some of the educational publications made using the Axebot.

It is worth mentioning that old UFBA students developed the researchers with the physical robot. Thus, this repository aims to provide a simulation version of the robot to ease the first steps of algorithms validation in the research process.

## Table of Contents

- [Installation Premises](#installation-premises)
- [Installing](#installing)
  - [Dependencies](#dependencies)
  - [Installing from source](#installing-from-source)
- [Building](#building)
- [Usage](#usage)
  - [Move to point](#move-to-point)
- [Contributing](#contributing)
- [Developed Research](#developed-research)

## Installation Premises

1. This repository has been tested in [ROS2 Foxy] and with [Classic Gazebo 11];

2. The installation steps below begin with the premise that you already installed the ROS2 Foxy Fitzroy in your machine. If this is not the case, see their instructions in the [recommended ubuntu installation tutorial]

3. If you don't already have an ament workspace, create it following the instructions in the [Creating a workspace tutorial]. After workspace creation, clone this repository in the source folder of your workspace
Installation

## Installing

### Dependencies

- [Omnidirectional Controllers](https://github.com/mateusmenezes95/omnidirectional_controllers)
- [Git LFS](https://git-lfs.com/)

### Installing from source

> **ATTENTION:** All the proposed commands in the remainder of this README assume you create the workspace with the name "ros_ws" in your home folder. Change the commands accordingly if you use another directory and/or name.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

``` 
cd ~/ros_ws/src
git clone https://github.com/mateusmenezes95/axebot.git
```

After that, install the dependencies of this package by running the following command in the root of your workspace:

```
cd ~/ros_ws
rosdep init
rosdep update
rosdep install --from-paths src/axebot --ignore-src -r -y --rosdistro foxy
```

If you already have all your dependencies, the console will return:

```
#All required rosdeps installed successfully
```

## Building

Run the command below to build the package.

```
cd ~/ros_ws
colcon build --symlink-install --event-handlers console_direct+
```

> Run `colcon build --help` to understand the arguments passed!

After the package building, source in a new terminal your workspace's  underlay and afterward the workspace overlay `local_setup.bash` to have the Axebot package available to use the ROS commands described in the next section.

```
source /opt/ros/foxy/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

> See [Source the overlay] to learn about underlay and overlay concepts.

## Usage

To launch the Gazebo simulation and spawn the robot into it, run the following:

```
ros2 launch axebot_gazebo axebot.launch.py
```

Thus, to move the robot in a straight line with a velocity of 0,5 m/s in the x-axis, execute:

```
ros2 topic pub \
omnidirectional_controller/cmd_vel_unstamped \
-r 10 \
geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

> You can also send twist velocity commands through the rqt publisher plugin by running the command `ros2 run rqt_publisher rqt_publisher` and then configuring the command in the GUI.

### Move to point

There is also a simple move-to-point node to drive the robot to a specific point (x, y) in the 2D cartesian space. To launch the node, run:

```
ros2 launch axebot_control move_axebot_to_point.launch.py
```

In a new terminal, command the robot to point (2, 2) executing:

```
ros2 topic pub /go_to_goal/goal --once geometry_msgs/msg/Vector3 "{x: 2.0, y: 2.0, z: 0.0}"
```

> Again, you may use the rqt_publisher to do that (recommended) and play with the Axebot when other 2D points are set.

This move-to-point node is a good starting point for someone that wants to develop high-level controllers to navigate the robot in an environment.

## Contributing

If you want to contribute to this package, you can [open an issue](https://github.com/mateusmenezes95/axebot/issues) describing the subject that you desire or develop the feature yourself and [open a pull request](https://github.com/mateusmenezes95/axebot/pulls) to the main branch (in this case, foxy).

In case of development by yourself, please, follow the [ROS 2 Code style and language] versions to leverage code reading and maintainability.

## Developed Research

J. Santos, A. Conceição, T. Santos, and H. Ara ujo, “Remote control of an omnidirectional mobile robot with time-varying delay and noise attenuation,” *Mechatronics*, vol. 52, pp. 7–21, 2018, ISSN: 0957-4158. DOI: 10.1016/j.mechatronics.2018.04.003. [Online](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300606).

J. Santos, A. G. Conceição, and T. L. Santos, “Trajectory tracking of omni-directional mobile robots via predictive control plus a filtered smith predictor,” IFAC- PapersOnLine, vol. 50, no. 1, pp. 10 250–10 255, 2017, 20th IFAC World Congress, ISSN: 2405-8963. DOI: 10.1016/j.ifacol.2017.08.1286. [Online](https://www.sciencedirect.com/science/article/pii/S2405896317318013).

J. C. L. Barreto S., A. G. S. Conceição, C. E. T. Dórea, L. Martinez and E. R. de Pieri, "Design and Implementation of Model-Predictive Control With Friction Compensation on an Omnidirectional Mobile Robot," in *IEEE/ASME Transactions on Mechatronics*, vol. 19, no. 2, pp. 467-476, April 2014, doi: 10.1109/TMECH.2013.2243161. [[link]](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300606)

A. G. S. Conceicao, M. D. Correia, and L. Martinez, “Modeling and friction estimation for wheeled omnidirectional mobile robots,” *Robotica*, vol. 34, no. 9, pp. 2140–2150, 2016. [[link]](https://www.cambridge.org/core/journals/robotica/article/abs/modeling-and-friction-estimation-for-wheeled-omnidirectional-mobile-robots/85796F5CF60310022D35CB792EF80254)

[Creating a workspace tutorial]: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace
[recommended ubuntu installation tutorial]: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
[Source the overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay
[ROS 2 Code style and language]: https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#code-style-and-language-versions
[ROS2 Foxy]: https://docs.ros.org/en/foxy/index.html
[Classic Gazebo 11]: https://classic.gazebosim.org/
