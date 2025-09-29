# Robotnik Gazebo Ignition

This package provides Gazebo Ignition plugins and resources for Robotnik robots.

## Launch Gazebo

Fist step to use this simulation is launch world where the robot will be spawned. For example, to launch the `empty` world, use the following command:

```bash
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

Available worlds are located in the `worlds` folder of this package. You can replace `empty` with the name of any other world file (without the `.world` extension) to launch a different world.

## Spawn Robot

Use the launch file to insert a robot into the Gazebo (Ignition) world.

### Basic
```bash
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbwatcher
```

### Advanced
```bash
# Specific ID and pose
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot_id:=robot_a robot:=rbwatcher robot_model:=rbwatcher x:=0.0 y:=0.0 z:=0.0 run_rviz:=true
```

```bash
# Generic pattern
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot_id:=<unique_name> robot:=<robot_type> robot_model:=<robot_model> x:=<m> y:=<m> z:=<m>
```

### Parameters
| Name | Required | Purpose | Example |
|---|---|---|---|
| `robot_id` | no | Instance name for the spawned robot | `robot_a` |
| `robot` | yes | Robot **type** to spawn | `rbwatcher` |
| `robot_model` | no | Specific **model** within the type | `rbwatcher` |
| `x` `y` `z` | no | Spawn position in meters | `0.0 0.0 0.0` |
| `run_rviz` | no | Launch RViz2 with a predefined configuration | `true` or `false` |

### Types vs. models
Description package is [robotnik_description](https://github.com/RobotnikAutomation/robotnik_description), which contains all robot types and models. The distinction is:
- **Robot type**: Category such as `rbwatcher`, `summit_xl`. See the package `robots/` folder for available types. [List of supported robots](https://github.com/RobotnikAutomation/robotnik_description/tree/jazzy-devel/robots).
- **Robot model**: Concrete variant inside a type. If omitted, the default model for that type is used. See the package `robots/<robot>/models/` folder for available models. [Example models for rbwatcher](https://github.com/RobotnikAutomation/robotnik_description/tree/jazzy-devel/robots/rbwatcher).

### Notes
- Use a unique `robot_id` when spawning multiple robots.

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
