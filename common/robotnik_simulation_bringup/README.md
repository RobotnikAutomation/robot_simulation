# Simulation

![alt text](docs/summit-gz.png)

## Bringup

Launch complete simulation

```
ros2 launch  robotnik_simulation_bringup bringup_complete.launch.py robot_model:=rbsummit use_gui:=true use_rviz:=false
```

#### Parameters
| Name | Required | Purpose | Example |
|---|---|---|---|
| `robot_id` | no | Name for launch and config resources | `robot` |
| `robot_model` | no | Name of the robot model | `rbsummit` |
| `use_gui` | no | Enable simulation graphical interface | `true` |
| `low_performance_simulation` | no | Enable smooth simulation for low performance computers | `true` |
| `use_rviz` | no | Launch rviz | `false` |
| `world_path` | no | Path of the world file | `/path/worlds/demo.world` |

Launch rviz for visualization:

```
ros2 launch  robotnik_simulation_bringup rviz.launch.py
```

![alt text](docs/summit-rviz.png)

## Create and use a new map

### 1. Run simulation

Launch the demo world:

```
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py gui:=true
```

Spawn the robot:

```
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py \
  robot:=rbsummit run_rviz:=false
```

In case of `rbsummit` or `rbwatcher` run Pointcloud to Laserscan node

```
ros2 launch robotnik_simulation_bringup laser_filters.launch.py
```

### 2. Autonomous navigation 

Run localization:

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

### 3. Mapping

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
map_url: '/home/robert/ws/sim/sim_ws/src/maps/test'
image_format: 'true'
map_mode: 'true'
free_thresh: 0.196
occupied_thresh: 0.65"
```