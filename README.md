# robot_simulation

Package for simulation of the Robotnik robots in ROS2.

## Structure

The structure of the package includes:

- debs: This folder includes all the dependencies for the simulation of the robots
- robot_description_simulation: This folder includes the urdf description of the robots and the gazebo controllers. Also, the robot_state_publisher launch is added here.
- robotnik_gazebo_classic: Package for the simulation of the robots in gazebo classic
- robotnik_gazebo_ignition: Package for the simulation of the robots in gazebo ignition


## How it works?

![img](/img/robotnik_simulation_structure.png)

### Launch A - Start Gazebo

The simulations available in this package are in Gazebo. So first, Gazebo has to start.

This launch initiates gzserver and gzclient, gzserver manages all the process in Gazebo and gzclient the visual interface.

### Launch B - Spawn Robot

Once the simulation has start, we can spawn a robot. For that there is a launch file that manage the nodes that the robot need to work.

#### 1. robot_state_publisher

The robot_state_publisher node reads the [URDF](/robot_description_simulation/robots/) and publish the robot_description topic.

| Subscribers  | Publishers        |
|--------------|-------------------|
| joint_states | robot_description |
| --           | tf                |
| --           | tf_static         |

#### 2. spawn_entity

This node is from Gazebo packages and it creates a robot model in the Gazebo world based on the robot_description information. Also, it starts the controller manager and launch the ros2_control plugin from Gazebo. After, the creation of the robot, this node dies.

| Subscribers       | Publishers |
|-------------------|------------|
| robot_description | --         |

#### 3. controller_manager

This node is from ros2_control package and it manage and configure the different controllers and interfaces. The interfaces are the movable joint designed in the robot_description.
The joints used for the interfaces are read from the [ros2_control URDF](/robot_description_simulation/ros2_control/classic/) that the robot_description has implemented.

The controller_manager has not any significant subscriber or publisher but it has important services that manage the controllers.

#### 4. ros_control

This node is the Gazebo plugin controller that generates the interfaces based on the URDF. This controller reads the interfaces command values that the robotnik_base_controller generates and commands them to the joints in Gazebo.

#### 5. robotnik_base_controller

This node is the Robotnik controller for the robots kinematics. It can control from an input of cartesian velocity or joints command directly and calculates the command interfaces values.

| Subscribers           | Publishers                     |
|-----------------------|--------------------------------|
| cmd_vel               | cmd_vel_limited                |
| joint_control_command | cmd_vel_previous_step          |
| imu/data              | controller_state               |
| emergency             | odom                           |
|                       | wheels_command_from_kinematics |
|                       | wheels_command_from_state      |
|                       | wheels_commands_output         |
|                       | wheels_previous_step           |

From the subscribers cmd_vel and joint_control_command it is calculated the wheels_command_output publihser, which are the values for the interfaces. Most of the publihsers are for evaluate the state of the controller.

The configuration of the controller can be found in this [config folder](/robotnik_gazebo_classic/config/).

#### 6. joint_state_broadcaster

This node is a controller that reads the states of the interfaces from the controller manager and publish the joint_states topic.

All this nodes are launched from the [spawn_robot launch](/robotnik_gazebo_classic/launch/spawn_robot.launch.py). Thanks to this, we can group all this nodes under the same namespace, beign able to spawn different robots in the Gazebo world.

> [!NOTE]
> At the moment, ros2_control is facing some issues handling different namespaces in Humble.

## Install

First install these packages:

```
git clone git@github.com:RobotnikAutomation/robotnik_common.git -b humble
git clone git@github.com:RobotnikAutomation/robot_description.git -b fix/ros2-devel/basic-robots
```

Then, install the debs files that includes the robotnik_controller and the robotnik_msgs

```
sudo dpkg -i debs/ros-humble-robotnik*.deb
```

## Gazebo Classic

## 1. Launch the gazebo world.

This launch initiates gazebo and the world.

```
ros2 launch robotnik_gazebo_classic spawn_world.launch.py
```

| Arguments  | Default    | Description                                                                                                 |
|------------|------------|-------------------------------------------------------------------------------------------------------------|
| world      | demo       | This arguments selects a world in  [ worlds ]( /robot_simulation/robotnik_gazebo_classic/worlds/ )  folder. |
| world_path | demo.world | In the case that you have your own world to simulate, introduce the path to the world file.                 |.

##### Examples:


```
ros2 launch robotnik_gazebo_classic spawn_world.launch.py world:=maze
```

## 2. Spawn the robot in gazebo world

This launch starts the robot_description publisher, spawn the robot in gazebo and starts the controllers.

```
ros2 launch robotnik_gazebo_classic spawn_robot.launch.py robot:=rbkairos
```
| Arguments        | Default                      | Description                                                                                    |
|------------------|------------------------------|------------------------------------------------------------------------------------------------|
| namespace        | robot                        | namespace that will be in the nodes and topics and differenciate one robot entity from another |
| robot            | rbkairos                     | robot type desired to be spawned                                                               |
| robot_model      | value of robot               | robot_model variation of the robot type. For using this argument, robot has to be fulfilled    |
| robot_xacro_path | rbkairos/rbkairos.urdf.xacro | path to a xacro model if it is not included in the robot_description_simulation package        |
| x                | 0.0                          | position x in the Gazebo world to spawn the robot                                              |
| y                | 0.0                          | position y in the Gazebo world to spawn the robot                                              |
| z                | 0.0                          | position z in the Gazebo world to spawn the robot                                              |


## 3. Control the robot

All the controllers for the robots work with a Twist topic called /namespace/robotnik_base_controller/cmd_vel, the default topic is:

```
/robot/robotnik_base_controller/cmd_vel
```

This topic will move the robot acsording to the velocity demanded but it can be also controller by joint commands, using the topic:

```
/robot/robotnik_base_controller/joint_control_command
```

This topic is from type sensor_msgs/msg/JointState.

I recommend to use teleop_twist_keyboard to control by cmd_vel:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-arg cmd_vel:=/robot/robotnik_base_controller/cmd_vel
```

## 4. Custom simulation process

In the case that the simulation it's not enough for your project, you can create your own simulation process.

### Steps

#### 1. Create a URDF file

First, start defining your URDF file. You can use the [robots files](/robot_description_simulation/robots/) to base your modifications on it, adding sensors or arms.

#### 2. Create a configuration for the controllers

Then, you will need to configure the robotnik_controller based on the topics that you want to use and the limitations. You will find the config files [here.](/robotnik_gazebo_classic/config/)

#### 3. Create the ros2_control URDF

Once you have the config controller, you will need to set the ros2_control for Gazebo, creating a new URDF. You can base on the examples [here.](/robot_description_simulation/ros2_control/)

In this files, you will have to modify the config path to the new one that you created in the point 2.

#### 4. Create a world

Create a world file if desired.

#### 5. Create a Launch file

This launch file will have to launch first the [spawn_world launch](/robotnik_gazebo_classic/launch/spawn_world.launch.py) with the arguments pointing to the custom world or a default one.

Then, it will have to launch the [spawn_robot launch](/robotnik_gazebo_classic/launch/spawn_world.launch.py). Here, you will have to modify the argument robot_xacro_path if you want to use your own URDF file.

#### 6. Try it

Once you changed all this, you will be able to use your custom simulation.