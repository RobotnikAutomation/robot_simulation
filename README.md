# robot_simulation

Package for simulation of the robotnik robots in ROS2.

The structure of the package includes:

- debs: This folder includes all the dependencies for the simulation of the robots
- robot_description_simulation: This folder includes the urdf description of the robots and the gazebo controllers. Also, the robot_state_publisher launch is added here
- robot_gazebo_classic: Package for the simulation for the robots in gazebo classic
- robot_gazebo_ignition: Package for the simulation for the robots in gazebo ignition

# How to use it?

First install these packages:

```
git clone git@github.com:RobotnikAutomation/robotnik_common.git -b humble
git clone git@github.com:RobotnikAutomation/robot_description.git -b fix/ros2-devel/basic-robots
```

## Gazebo Classic

## 1. Launch the gazebo world.

This launch initiates gazebo and the world.

```
ros2 launch robotnik_gazebo_classic spawn_world.launch.py
```

arguments for the launch:

- world: This arguments selects a world in [worlds](/robot_simulation/robotnik_gazebo_classic/worlds/) folder.

```
ros2 launch robotnik_gazebo_classic spawn_world.launch.py world:=maze
```

- world_path: In the case that you have your own world to simulate, introduce the path to the world file.

## 2. Spawn the robot in gazebo world

This launch starts the robot_description publisher, spawn the robot in gazebo and starts the controllers.

```
ros2 launch robotnik_gazebo_classic spawn_robot.launch.py robot:=rbkairos
```

arguments for the launch:

- namespace: This will add a namespace to the nodes and topics so we can diferenciate between robots.

- robot: select the robot to spawn, options: rbkairos, rbvogui, rbsummit, rbtheron.

## 3. Control the robot

All the controllers for the robots work with a TwistStamped topic called /namespace/robotnik_base_controller/cmd_vel, the default topic is:

```
/robot/robotnik_base_controller/cmd_vel
```

In case of the RBKairos, the mecanum wheel cannot be simulated correctly in gazebo so move planar is used to control the robot, in that case the topic is Twist:

```
/robot/cmd_vel_limited
```
