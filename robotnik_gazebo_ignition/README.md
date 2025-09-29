# Robotnik Gazebo Ignition

This package provides Gazebo Ignition plugins and resources for Robotnik robots.

## Launch Gazebo

Fist step to use this simulation is launch world where the robot will be spawned. For example, to launch the `empty` world, use the following command:

```bash
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

Available worlds are located in the `worlds` folder of this package. You can replace `empty` with the name of any other world file (without the `.world` extension) to launch a different world.

## Spawn Robot

Once you have the simulation up and running, you can spawn a robot into the Gazebo environment. For that, there is a launch file that starts all the nodes.

```bash
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbwatcher
```

Replace `<robot_name>` with the name of the robot you want to spawn (e.g., `turtlebot3_burger`, `turtlebot3_waffle`, etc.) and `<namespace>` with the desired namespace for the robot (e.g., `tb3_0`, `tb3_1`, etc.).

| Parameter | Description | Default |
| --------- | ----------- | ------- |
| `robot_id` | Unique identifier for the robot, useful when spawning multiple robots | `robot` |
| `robot` | Robot type desired to be spawned, no default value | None |
| `robot_model` | Robot model, if not specified, it will be set according to the `robot` parameter | _same as robot_ |
| `x` | X position where the robot will be spawned | `0.0` |
| `y` | Y position where the robot will be spawned | `0.0` |
| `z` | Z position where the robot will be spawned | `0.0` |


## Control the Robot

After spawning the robot, you can control it using command velocity messages. The two main topics for controlling the robot are:
- `/<robot-id>/robotnik_base_control/cmd_vel`: This topic is used to send velocity commands to the robot. The messages should be of type `geometry_msgs/msg/TwistStamped`.
- `/<robot-id>/robotnik_base_control/cmd_vel_unstamped`: This topic is used to send velocity commands without a timestamp. The messages should be of type `geometry_msgs/msg/Twist`.


To control the robot, you can use teleoperation packages such as `teleop_twist_keyboard` or `teleop_twist_joy`. For example, to control the robot using the keyboard, run:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot/robotnik_base_control/cmd_vel -p stamped:=true
```

Make sure to replace `/robot/robotnik_base_control/cmd_vel` with the appropriate topic name based on the `robot_id` you used when spawning the robot.

Also, you can use RViz plugin on the bottom right to control the robot by clicking on the arrows.
