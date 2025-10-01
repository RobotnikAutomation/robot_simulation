# Robotnik Gazebo Ignition

This package provides Gazebo Ignition plugins and resources for Robotnik robots.

> **⚠️ Prerequisites**: Make sure to complete the [Installation](#installation) section before running any of the commands below.

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


## Installation

1. Setup sources and keys.
```sh
sudo apt update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

2. Install Gazebo Harmonic.
```sh
sudo apt-get update
sudo apt-get install gz-harmonic
```

3. Install ROS 2 Jazzy and ROS-GZ bridge.
```sh
sudo apt install ros-jazzy-ros-gz
```

4. Set up workspace and install dependencies:

```sh
# Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Robotnik and related packages (ROS 2 Jazzy)
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/robotnik_simulation/jazzy-devel/robotnik_simulation.jazzy.repos src/

# Install prebuilt simulation debs from this repo (run at repo root)
cd ~/ros2_ws/src/robotnik/robotnik_simulation
sudo apt-get install -y ./debs/ros-${ROS_DISTRO}-*.deb

# Resolve dependencies
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

5. Build the workspace:

```sh
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Enjoy

Example of RBVogui executing docking procedure in Gazebo Ignition:

![rbvogui_gif](../docs/assets/img/RBVogui_Docking.gif)

## Custom robot model

1. Create a new package for your project.
2. Create a URDF/XACRO. Use the template in `robotnik_description` as a starting point:
   `robotnik_description/robots/robot_template.urdf.xacro`
3. See `robotnik_description/README.md` for a brief guide to composing robots.
4. Add sensors, arms, and components as needed.
5. Spawn with `robot_xacro_path`:

```sh
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot_xacro_path:=<your_robot.urdf.xacro>
```

## Custom control

Edit the robot-specific config files in:
`robotnik_gazebo_ignition/config/`
You can adjust topics, frames, velocity limits, and controllers.

### Custom world

Pass a custom world file via `world_path`:

```sh
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world_path:=<your_world.sdf>
```

## Docker
🚧 Work in progress. 🚧

Use the compose file in the repo root to run a preconfigured simulator container.

```sh
docker compose up
```

> **Note**: The first time will take a while as it builds the image. Subsequent runs will be faster.
