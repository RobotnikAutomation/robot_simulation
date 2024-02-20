# robot_advanced_sim

## Package Description

`robot_advanced_sim` is a ROS package for advanced simulation of robots. This package provides a comprehensive set of tools and libraries to simulate complex robotic systems in a variety of environments.

## Setup

To install `robot_advanced_sim`, clone the repository into your catkin workspace and build it using catkin.

```bash
cd ~/catkin_ws/src
git clone https://github.com/RobotnikAutomation/robot_advanced_sim.git
cd ..
catkin_build
```

## Usage

If the env variable ROBOT_MODEL is defined, the value will be taken in the robot_complete.launch:

```bash
roslaunch robot_advanced_sim robot_complete.launch
```

Otherwise, there are launch files for every robot model:

```bash
roslaunch robot_advanced_sim summit_xl_complete.launch
```

Note: some of the launch arguments are taken from the Robotnik's standard env variables defined in the package robot_bringup.