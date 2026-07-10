# Robotnik Simulation Bringup
This package provides high-level launch files for complete Robotnik simulation demos.

It is intended to orchestrate several simulation components together:

- Gazebo world and robot spawning
- robot-specific laser filtering when required
- localization
- navigation
- RViz visualization
- optional MoveIt integration for mobile manipulators

For a simple Gazebo-only simulation with one robot, use the launch files in robotnik_gazebo_ignition. This package is intended for full demo bringup flows where navigation, localization or manipulation are required.

![alt text](docs/summit-gz.png)

## Quick start
### Full navigation demo 

Launch the default simulation stack (with RBSummit):

```
source ~/ros2_ws/install/setup.bash
ros2 launch  robotnik_simulation_bringup bringup_complete.launch.py
```

Or launch the default simulation with rbwatcher:
```
source ~/ros2_ws/install/setup.bash
ros2 launch  robotnik_simulation_bringup bringup_complete.launch.py robot:=rbwatcher
```

Or launch complete RbKairos simulation with MoveIt enabled:

```
ros2 launch robotnik_simulation_bringup bringup_complete.launch.py robot:=rbkairos robot_model:=rbkairos_plus arm_type:=ur10e use_gui:=true use_rviz:=true run_moveit:=true
```

Launch complete simulation with MoveIt and custom xacro path:

```
ros2 launch robotnik_simulation_bringup bringup_complete.launch.py robot:=rbkairos robot_model:=rbkairos_plus robot_xacro_path:=/path/to/robot.urdf.xacro arm_type:=ur10e use_gui:=true use_rviz:=true run_moveit:=true
```

#### Parameters
| Name | Required | Default | Purpose |
|---|---|---|---|
| `robot_id` | no | `robot` | Name for launch and config resources |
| `robot` | no | `rbsummit` | Robot base type used to resolve defaults |
| `robot_model` | no |  value of 'robot' | Name of the robot model |
| `robot_xacro_path` | no | resolved from `robot` and `robot_model` | Path to robot URDF/XACRO (forwarded to spawn and MoveIt) |
| `use_gui` | no | `true` | Enable simulation graphical interface |
| `low_performance_simulation` | no | `true` | Enable smooth simulation for low performance computers |
| `use_rviz` | no | `true` | Launch rviz |
| `run_localization` | no | `true` | Launch AMCL |
| `run_navigation` | no | `true` | Launch nav2 |
| `run_moveit` | no | `false` | Launch MoveIt for supported mobile manipulators |
| `run_laser_filters` | no | `true` | Launch laser filters when supported by the selected robot model. |
| `arm_type` | no | `ur10e` | Arm type used for robots with manipulator (forwarded as xacro `ur_type`) |
| `world_path` | no | `/robotnik_gazebo_ignition/worlds/demo.world` | Path of the world file |

## MoveIt

MoveIt can be launched in two modes:

### Integrated in complete bringup:

```
ros2 launch robotnik_simulation_bringup bringup_complete.launch.py robot:=rbkairos robot_model:=rbkairos_plus arm_type:=ur10e run_moveit:=true
```

Integrated in bringup with custom xacro path:

```
ros2 launch robotnik_simulation_bringup bringup_complete.launch.py robot:=rbkairos robot_model:=rbkairos_plus robot_xacro_path:=/path/to/robot.urdf.xacro arm_type:=ur10e run_moveit:=true
```

### Independently from the simulation bringup pipeline (use this after the robot and controllers are already running):

```
ros2 launch robotnik_simulation_moveit moveit.launch.py robot_id:=robot robot:=rbkairos robot_model:=rbkairos_plus moveit_config_name:=rbkairos_moveit_config arm_type:=ur10e run_moveit_rviz:=true
```

Independently with custom xacro path:

```
ros2 launch robotnik_simulation_moveit moveit.launch.py robot_id:=robot robot:=rbkairos robot_model:=rbkairos_plus robot_xacro_path:=/path/to/robot.urdf.xacro moveit_config_name:=rbkairos_moveit_config arm_type:=ur10e run_moveit_rviz:=true
```

### rviz only for visualization:

```
ros2 launch  robotnik_simulation_bringup rviz.launch.py
```

![alt text](docs/summit-rviz.png)

## Create and use a new map

### 1. Launch simulation

For mapping workflows, you can start the simple simulation:

```
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py gui:=true
```

Spawn the robot:

```
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py \
  robot:=rbsummit run_rviz:=false
```

If required by the selected robot (rbsummit or rbwatcher), launch the laser filters:

```
ros2 launch robotnik_simulation_bringup laser_filters.launch.py
```

### 2. Launch SLAM algorithm (Mapping)

Run mapping:

```
ros2 launch robotnik_simulation_localization localization.launch.py run_mapping:=true
```

Start mapping:

```
ros2 service call /robot/lifecycle_manager_mapping/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "command: 0"
```

Save map:

```
ros2 service call /robot/map_saver/save_map nav2_msgs/srv/SaveMap "map_topic: '/robot/map'
map_url: '/home/robot/maps/demo_map/demo_map'
image_format: 'png'
map_mode: 'trinary'
free_thresh: 0.196
occupied_thresh: 0.65"
```

### 3. Launch Autonomous Navigation with your map

Stop the mapping launch before starting localization:

```
ros2 launch robotnik_simulation_localization localization.launch.py
```

Load map:

```
ros2 service call /robot/map_server/load_map nav2_msgs/srv/LoadMap "map_url: '/home/robot/maps/demo_map/demo_map.yaml'" 
```

Run navigation:

```
ros2 launch robotnik_simulation_navigation navigation.launch.py
```
