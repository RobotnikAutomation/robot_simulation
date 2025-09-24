<a id="readme-top"></a>


<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Unlicense License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


<!-- PROJECT LOGO -->
![Logo Robotnik](./docs/assets/img/LOGO%20BLANCO-ROJO.png)

<br />
<div align="center">
=======

  <h1 align="center">robot_simulation</h1>

  <p align="center">
    Simulation of Robotnik Automation robots in ROS2!
    <br />
    <a href="https://github.com/RobotnikAutomation/robot_simulation"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="#enjoy">View Demo</a>
    &middot;
    <a href="https://github.com/RobotnikAutomation/robot_simulation/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    &middot;
    <a href="https://github.com/RobotnikAutomation/robot_simulation/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
        <a href="#usage">Usage</a>
      <ul>
        <li>
            <a href="#gazebo-ignition">Gazebo Ignition</a>
        <ul>
            <li><a href="#launch-gazebo">Launch Gazebo</a></li>
            <li><a href="#spawn-robot">Spawn Robot</a></li>
            <li><a href="#control-the-robot">Control the Robot</a></li>
        </ul>
        </li>
      </ul>
    </li>
    <li>
        <a href="#custom-simulation">Custom Simulation</a>
      <ul>
        <li><a href="#custom-robot-model">Custom Robot Model</a></li>
        <li><a href="#custom-world">Custom World</a></li>
        <li><a href="#custom-control">Custom Control</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Simulation View][product-screenshot]](https://github.com/RobotnikAutomation/robot_simulation)

This package will combine the different Robotnik packages in ROS2 to simulate the robots in the different available platforms, as Gazebo Sim 8.9.0, etc.

This README will guide you to the simulation usage and the custom simulations that you can build for your own projects.



<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

This package works with the different packages that Robotnik developed for the robots. You will need to install some of these packages first.

### Prerequisites

#### - Gazebo Sim 8.9.0
#### - ROS2 Jazzy

First, be sure that you have all the [Gazebo packages](https://gazebosim.org/docs/harmonic/install_ubuntu/) installed for ROS2.


```sh
sudo apt install ros-jazzy-ros-gz
```

### Installation

Then, let's procede with the installation of the Robotnik packages.
First, create the workspace to work with:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
```
Then continue with the installation of packages.

1. [robotnik_description](https://github.com/RobotnikAutomation/robotnik_description/tree/jazzy-devel)
2. [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors/tree/jazzy-devel)
3. [robotnik_common](https://github.com/RobotnikAutomation/robotnik_common)
4. [robotnik_interfaces](https://github.com/RobotnikAutomation/robotnik_interfaces)
5. [ur_description](https://github.com/RobotnikAutomation/Universal_Robots_ROS2_Description/tree/fix/gazebo-control-jazzy#)
6. [robotnik_simulation](https://github.com/RobotnikAutomation/robotnik_simulation.git#)

  ```sh
  git clone https://github.com/RobotnikAutomation/robotnik_description.git -b jazzy-devel

  git clone https://github.com/RobotnikAutomation/robotnik_sensors.git -b jazzy-devel

  git clone https://github.com/RobotnikAutomation/robotnik_common.git -b ros2-devel

  git clone https://github.com/RobotnikAutomation/robotnik_interfaces.git -b jazzy-devel

  git clone https://github.com/RobotnikAutomation/Universal_Robots_ROS2_Description.git -b fix/gazebo-control-jazzy

  git clone https://github.com/RobotnikAutomation/robotnik_simulation.git -b jazzy-devel

  ```

Install precompiled debs for simulation. Please, change directory to the root of the repository and run the following command:

```sh
sudo apt-get install -y ./debs/ros-${ROS_DISTRO}-*.deb
```

Install missing dependencies with rosdep:
```sh
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, compile workspace:
```
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build
source install/setup.bash
```


<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Docker
If you want to use Docker, you can use the docker-compose file that is in the root of the repository. This will create a container with all the dependencies installed and ready to use.

To start the containers, run the following command:

```sh
docker compose up
```
This will build a robotnik_simulator image the first time you run it. After that, it will use the cached image.

### Setup
You can configure the simulation based on docker images editing the environment in the env/robot.env file. You will need to uncomment the specific variables related to desired robot to simulate and comment or delete the others.

Make sure that the environment variables (ROBOT, ROBOT_MODEL, HAS_ARM) are set correctly before running the containers.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

### Gazebo Ignition

#### Launch Gazebo

Init the Gazebo world by launching:

```sh
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py
```

#### Spawn Robot

Once you have the simulation running, you can spawn the robot in the world. For that, there is a launch file that starts all the nodes.

| Arguments        | Default                      | Description                                                                                    |
|------------------|------------------------------|------------------------------------------------------------------------------------------------|
| namespace        | robot                        | namespace that will be in the nodes and topics and differenciate one robot entity from another |
| robot            | ''                     | robot type desired to be spawned, must be specified                                                  |
| robot_model      | _same as robot_              | robot_model variation of the robot type. For using this argument, robot has to be fulfilled    |
| robot_xacro_path | rbkairos/rbkairos.urdf.xacro | path to a xacro model if it is not included in the robotnik_description package                |
| x                | 0.0                          | position x in the Gazebo world to spawn the robot                                              |
| y                | 0.0                          | position y in the Gazebo world to spawn the robot                                              |
| z                | 0.0                          | position z in the Gazebo world to spawn the robot                                              |

With the arguments described above, the launcher creates the robot that you want in Gazebo. As default, it will spawn a RBKairos robot, but you can changed it.

Available robots

- rbvogui
- rbtheron
- rbsummit
- rbkairos
- rbrobout
- rbwatcher
- rbfiqus

Available robot_model

- rbkairos_plus
- rbrobout_plus

Examples:
```sh
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbvogui

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbtheron

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbsummit

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbkairos robot_model:=rbkairos

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbrobout robot_model:=rbrobout

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbwatcher robot_model:=rbwatcher

ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbfiqus robot_model:=rbfiqus
```

Additionally, the arguments _x_, _y_ and _z_ selects the position respect the world frame to spawn the robot.

In case that your robot has a variation (check robots folder in robotnik_description package), you can select it by the argument **robot_model**.


#### Control the robot

All the controllers for the robots work with a TwistStamped topic called /namespace/robotnik_base_controller/cmd_vel, the default topic is:

```sh
/robot/robotnik_base_controller/cmd_vel
```
Also it can be used a Twist topic:
```sh
/robot/robotnik_base_controller/cmd_vel_unstamped
```

This topic will move the robot acording to the velocity demanded but it can be also controller by joint commands, using the topic:

```sh
/robot/robotnik_base_controller/cmd_joint
```

Topic type sensor_msgs/msg/JointState.

I recommend to use teleop_twist_keyboard to control by cmd_vel:

```sh
sudo apt install ros-jazzy-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot/robotnik_base_controller/cmd_vel -p stamped:=true
```

### Mobile robots with manipulators

There are two mobile bases with a manipulator that can be used:
- rbkairos_plus
- rbrobout_plus

To use them launch the spawn of the robot as follows:

```sh
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbkairos robot_model:=rbkairos_plus
```


```sh
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbrobout robot_model:=rbrobout_plus
```

The arm has a joint_trajectory_controller configured that can be used with rqt_joint_trajectory_controller:


```sh
sudo apt install ros-jazzy-rqt-joint-trajectory-controller

ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --ros-args -r __ns:=/robot
```

#### Enjoy!

![rbvogui_gif](docs/assets/img/RBVogui_Docking.gif)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CUSTOMIZATION -->
## Custom Simulation

In case that the robot model that you want to simulate is not in robotnik_description package, or the world, or you want to use a different controller, you will have to create your own simulaiton files.

This will guide you to create your custom simulation.

### Custom Robot Model

1. First, create your own package for the project.
2. In this package, create your URDF file. You can base in the [template file](robotnik_pkgs/robotnik_description/robots/robot_template.urdf.xacro) that are in robotnik_description package.
3. In the README of [robotnik_description](robotnik_pkgs/robotnik_description/README.md) there is a brief descripiton on how to create a robot.
4. On this file you can modify and add all the sensors, arms and any other component.
5. Then, launch the spawn_robot with the argument _robot_xacro_path_.

### Custom Control

In case that you want to modify the velocity, topics, frames and everything related to the control, you can find the files in [robotnik_gazebo_ignition/config folder](robotnik_gazebo_ignition/config/).
There is a file for each robot that you can modify.

### Custom world

To launch a custom file, you can use the _world_path_ argument that it's in the spawn_world launch. See <a href="#launch-gazebo">Launch Gazebo</a>.

<!-- ROADMAP -->
## Roadmap

- [x] Add Gazebo Ignition
- [ ] Add more worlds
- [ ] Add multi robot support

See the [open issues](https://github.com/RobotnikAutomation/robot_simulation/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Top contributors:

<a href="https://github.com/RobotnikAutomation/robot_simulation/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=RobotnikAutomation/robot_simulation" alt="contrib.rocks image" />
</a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the Unlicense License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Project Link: [https://github.com/RobotnikAutomation](https://github.com/RobotnikAutomation)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/RobotnikAutomation/robot_simulation.svg?style=for-the-badge
[contributors-url]: https://github.com/RobotnikAutomation/robot_simulation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/RobotnikAutomation/robot_simulation.svg?style=for-the-badge
[forks-url]: https://github.com/RobotnikAutomation/robot_simulation/network/members
[stars-shield]: https://img.shields.io/github/stars/RobotnikAutomation/robot_simulation.svg?style=for-the-badge
[stars-url]: https://github.com/RobotnikAutomation/robot_simulation/stargazers
[issues-shield]: https://img.shields.io/github/issues/RobotnikAutomation/robot_simulation.svg?style=for-the-badge
[issues-url]: https://github.com/RobotnikAutomation/robot_simulation/issues
[license-shield]: https://img.shields.io/github/license/RobotnikAutomation/robot_simulation.svg?style=for-the-badge
[license-url]: https://github.com/RobotnikAutomation/robot_simulation/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/robotnik-automation/
[product-screenshot]: docs/assets/img/ignition_simulation_view.png
