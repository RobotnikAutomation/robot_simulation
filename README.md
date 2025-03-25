# robot_sim

## Package Description

`robot_sim` is a ROS package for advanced simulation of robots. This package provides a comprehensive set of tools and libraries to simulate complex robotic systems in a variety of environments.

## Simulation

This simulation has been tested using Gazebo 11 version.

## Installation

This simulation works in `ROS noetic` and uses `rviz` and `gazebo`.

### 1. Install the following dependencies:

```bash 
sudo apt update && sudo apt-get install python3-rosdep python3-vcstool git python3-catkin-tools python-is-python3
```

### 2. Create a workspace and clone the repository:

```bash
mkdir ~/catkin_ws && cd ~/catkin_ws
```

Install the latest version of the simulation:

```bash
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/robotnik_simulation/refs/heads/ros-devel/repos/robot_sim.repos
```

**Install the private packages and the ROS dependencies:**

```bash
sudo ./src/robot_sim/lib/install_debs.sh
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -y -r
```

### 3. Compile

```bash
catkin_build
source devel/setup.bash
```

## Usage

### 1. Simulation of a robot
If the env variable `ROBOT_MODEL` is defined, the value will be taken in the `robot_complete.launch`:

```bash
roslaunch robot_sim robot_complete.launch
```

Otherwise, there are launch files for every robot model:

```bash
roslaunch robot_sim <robot_model>_complete.launch
```

Or the argument `robot_model` can be set in `robot_complete.launch`:

```bash
roslaunch robot_sim robot_complete.launch robot_model:=<robot_model>
```

*Note: some launch arguments are taken from the Robotnik's standard env variables defined in the package robot_bringup.*

Substitute *<robot_model>* with one of the following models:
- summit_xl
- summit_xl_steel
- summit_hm
- rbwatcher
- rbvogui
- rbvogui_xl
- rbvogui_6w
- rb_theron
- omnitheron
- rbsherkan
- rbrobout
- rbkairos
- rb1_base

##### 1.1. Launch arguments of interest
**robot_complete.launch**

```xml
<arg name="robot_model" default="$(optenv ROBOT_MODEL summit_xl)"/>
<arg name="robot_id" default="$(optenv ROBOT_ID robot)"/>

<!-- Loc & Nav -->
<arg name="launch_loc_nav" default="true"/>

<!-- Description -->
<arg name="robot_xacro" default="$(optenv ROBOT_XACRO)"/>
<arg name="robot_xacro_package" default="$(optenv ROBOT_XACRO_PACKAGE)"/>
<arg name="robot_xacro_relative_path" default="$(optenv ROBOT_XACRO_RELATIVE_PATH)" />
<arg name="robot_xacro_folder" default="$(optenv ROBOT_XACRO_FOLDER)" />
```

- `robot_model`: Defines the robot to simulate.

- `robot_id`: ROS namespace of the simulation.

- `launch_loc_nav`: Flag to run localization and navigation nodes.

- `robot_xacro_folder`: If the xacro file is not in a ROS package, here can be defined the folder that contains it. If it is in a ROS package, set this argument as *none*.
    - If the argument is not set, the default value in `<robot_model>_complete.launch` will be used (*none*).

- `robot_xacro_package`: ROS package that contains the xacro file.
    - If the argument is not set, the default value in `<robot_model>_complete.launch` will be used (*robot_description*).

- `robot_xacro_relative_path`: Folder inside `robot_xacro_package` that contains the xacro file.
    - If the argument is not set, the default value in `<robot_model>_complete.launch` will be used (*/robots/*).

- `robot_xacro`: Name of the xacro file.
    - If the argument is not set, the default value in `<robot_model>_complete.launch` will be used.

Example: To simulate a rbvogui without navigation and localization:
```bash
roslaunch robot_sim robot_complete.launch robot_model:=rbvogui launch_loc_nav:=false
```
**<robot_model>_complete.launch**

Example: `rbkairos_complete.launch`

```xml
<arg name="robot_model" default="rbkairos"/>
<!-- arg to config the launch file-->
<arg name="robot_id" default="$(optenv ROBOT_ID robot)"/>

<!-- Simulation -->
<arg name="sim_world" default="$(optenv ROBOT_SIM_WORLD willow_garage)"/>
<arg name="x_init_pose" default="$(optenv ROBOT_SIM_X_POSE 0)"/>
<arg name="y_init_pose" default="$(optenv ROBOT_SIM_Y_POSE 0)"/>
<arg name="z_init_pose" default="$(optenv ROBOT_SIM_Z_POSE 0)"/>
<arg name="a_init_pose" default="$(optenv ROBOT_SIM_Z_ANGLE 0)"/>
<arg name="spawn_world" default="true"/>

<!-- Description -->
<arg name="robot_xacro" default="rbkairos_ur10e_plus.urdf.xacro"/>
<arg name="robot_xacro_package" default="robot_description"/>
<arg name="robot_xacro_relative_path" default="/robots/" />
<arg name="robot_xacro_folder" default="none" />
<arg if="$(eval robot_xacro_folder =='none')" name="robot_xacro_path" default="$(eval find(robot_xacro_package) + robot_xacro_relative_path + robot_xacro)"/>
<arg unless="$(eval robot_xacro_folder =='none')" name="robot_xacro_path" default="$(arg robot_xacro_folder)/$(arg robot_xacro)"/>

<!-- Safety -->
<arg name="has_safety_module" default="false" />

<!-- Loc & Nav -->
<arg name="launch_loc_nav" default="true"/>
<arg name="launch_nav" default="$(arg launch_loc_nav)"/>
<arg name="launch_mapserver" default="$(arg launch_loc_nav)"/>
<arg name="launch_amcl" default="$(arg launch_loc_nav)"/>
```

Apart from the arguments listed for `robot_complete.launch`, there are other arguments that can be of interest:

- `sim_world`: World to spawn in the simulation. This world must have a launch file in `robot_sim/launch/gazebo/world/`. 

- `x_init_pose`/`y_init_pose`/`z_init_pose`/`a_init_pose`: Init pose of the robot in the x, y and z axes and also in orientation.

- `robot_xacro_path`: Complete path to the xacro file of the robot model. This argument can be set with the xacro arguments listed for `robot_complete.launch`.

- `has_safety_module`: Flag to simulate a safety module.

Example: To simulate a rbrobout without navigation and localization with safety module in the world *demo*:
```bash
roslaunch robot_sim rbrobout_complete.launch launch_loc_nav:=false has_safety_module:=true sim_world:=demo
```

### 2. Add robot to simulation

To add a robot to an existing simulation, run:

```bash
roslaunch robot_sim add_robot_to_simulation.launch
```

This launch file has the same arguments as `<robot_model>_complete.launch`.

```xml
<arg name="robot_model" default="$(optenv ROBOT_MODEL summit_xl)"/>
<arg name="robot_id" default="robot2"/>

<!-- Loc & Nav -->
<arg name="launch_loc_nav" default="true"/>

<!-- Init pose -->
<arg name="x_init_pose" default="2"/>
<arg name="y_init_pose" default="2"/>
<arg name="z_init_pose" default="0"/>
<arg name="a_init_pose" default="2"/>

<!-- Safety -->
<arg name="has_safety_module" default="false" />

<!-- Description -->
<arg name="robot_xacro" default=""/>
<arg name="robot_xacro_package" default=""/>
<arg name="robot_xacro_relative_path" default=""/>
<arg name="robot_xacro_folder" default=""/>
```

Example: Run the simulation of a summit_xl and add a rb_theron to the simulation:

Terminal 1:
```bash
roslaunch robot_sim summit_xl_complete.launch
```

Terminal 2:
```bash
roslaunch robot_sim add_robot_to_simulation.launch robot_model:=rb_theron
```


WARNING: For each robot added to the simulation, a new `robot_id` has to be set. The default value is *robot2*. Also, make sure the init poses of the robots are different to prevent collisions.
![image](https://github.com/user-attachments/assets/1cb2fd34-a3d6-457c-bf7a-74ea9b9ee686)

