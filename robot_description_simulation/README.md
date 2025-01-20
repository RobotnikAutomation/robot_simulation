# robot_description_simulation

This package is an update version of the [robot_description](https://github.com/RobotnikAutomation/robot_description/tree/fix/ros2-devel/basic-robots) that includes the gazebo plugins and controllers.

## Package Structure

This package is divided in 2 main folders:

- robots: This folder is an extension of the [robots folder of robot_description](https://github.com/RobotnikAutomation/robot_description/tree/fix/ros2-devel/basic-robots/robots). Here we will find the urdf files that includes the base, sensors and the ros2_control gazebo plugins that are defined in the next folder.
- ros2_control: This folder includes more urdf files that define ros2_control plugins for gazebo simulation.

## Robot Structure

Comparing robot_description and robot_description_simulation the structures are as follow:

![img](/robot_description_simulation/img/robot_description_simulation.matriuska.png)

As you can see in the image, the robot_description_simulation urdfs that are in [robots folder](/robot_description_simulation/robots/) are a copy of the [robots folder of robot_description](https://github.com/RobotnikAutomation/robot_description/tree/fix/ros2-devel/basic-robots/robots) but adding the ros2_control urdf file that are in [ros2_control](/robot_description_simulation/ros2_control/).

## Launch

As in [robot_description](https://github.com/RobotnikAutomation/robot_description/tree/fix/ros2-devel/basic-robots), the launch publish the robot_description running the robot_state_publisher, adding 2 new arguments.

|  argument | default  | definition  |
|---|---|---|
| namespace  |  robot | add namespace to the node and topic (/robot/robot_state_publisher and /robot/robot_description)  |
|  robot | rbvogui  | which robot select from folder robots  |
|  robot_model | robot argument value  | this argument is used to select the specific version of the robot. Example, on rbkairos folder there are 2 versions, rbkarios and rbkairos_ur. To use this argument correctly, the previous argument has to be used also.  |
|  robot_xacro_path | path to the urdf file to use | In case of using a custom robot that it is not in robot_description, select the path to the file in this argument|
|  use_gazebo_classic | false | This arguments turn true the controllers and plugins for gazebo classic simulation|
|  use_gazebo_ignition | false | This arguments turn true the controllers and plugins for gazebo ignition simulation|