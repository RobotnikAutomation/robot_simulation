<a id="readme-top"></a>

<!-- SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- LOGO -->
<p align="center">
  <img src="./docs/assets/img/LOGO%20BLANCO-ROJO.png" alt="Robotnik logo" height="80">
</p>

<h1 align="center">Robotnik - ROS 2 Simulation</h1>

<p align="center">
  ROS 2 simulation of Robotnik Automation robots.
  <br />
  <a href="https://github.com/RobotnikAutomation/robot_simulation"><strong>Browse the docs »</strong></a>
  <br /><br />
  <a href="#demo">View demo</a>
  ·
  <a href="https://github.com/RobotnikAutomation/robot_simulation/issues/new?labels=bug&template=bug-report---.md">Report bug</a>
  ·
  <a href="https://github.com/RobotnikAutomation/robot_simulation/issues/new?labels=enhancement&template=feature-request---.md">Request feature</a>
</p>

---

## Table of contents
- [About](#about)
- [Quick start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Install](#install)
  - [Build](#build)
- [Usage](#usage)
- [Docker](#docker)
  - [Setup](#setup)
- [Custom simulation](#custom-simulation)
  - [Custom robot model](#custom-robot-model)
  - [Custom control](#custom-control)
  - [Custom world](#custom-world)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Demo](#demo)

---

## About
This repository provides Gazebo Sim–based environments and launch assets for simulating Robotnik platforms in ROS 2. It also points to the required Robotnik packages used across simulations.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Quick start

### Prerequisites
- ROS 2 **Jazzy**
- Gazebo **Sim Harmonic 8.9.0** (a.k.a. Ignition)
- `ros_gz` bridge

Install Gazebo Sim and bridge (Ubuntu):
```sh
sudo apt update
sudo apt install ros-jazzy-ros-gz
````

### Install

Create a workspace and clone needed packages:

```sh
# Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Robotnik and related packages (ROS 2 Jazzy)
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/robotnik_simulation/jazzy-devel/robotnik_simulation.jazzy.repos src/
```

Install prebuilt simulation debs from this repo (run at repo root):

```sh
cd ~/ros2_ws/src/robotnik_simulation
sudo apt-get install -y ./debs/ros-${ROS_DISTRO}-*.deb
```

Resolve dependencies:

```sh
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```sh
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage

See each package for details:

* [`robotnik_gazebo_ignition`](robotnik_gazebo_ignition/README.md)

Typical flow:

```sh
# Launch Gazebo with a world
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world_path:=<path/to/world.sdf>

# Spawn a robot (example)
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot_xacro_path:=<path/to/robot.urdf.xacro>

# Control example (publish to cmd_vel)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Docker

Use the compose file in the repo root to run a preconfigured simulator container.

```sh
docker compose up
```

The first run builds the `robotnik_simulator` image. Subsequent runs reuse the cache.

### Setup

Configure the environment via `env/robot.env`. Set:

* `ROBOT`
* `ROBOT_MODEL`
* `HAS_ARM`

Uncomment only the variables for the robot you want to simulate.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Custom simulation

If your robot, world, or controller is not provided, create a thin overlay package and supply custom assets.

### Custom robot model

1. Create a new package for your project.
2. Create a URDF/XACRO. Use the template in `robotnik_description` as a starting point:
   `robotnik_description/robots/robot_template.urdf.xacro`
3. See `robotnik_description/README.md` for a brief guide to composing robots.
4. Add sensors, arms, and components as needed.
5. Spawn with `robot_xacro_path`:

   ```sh
   ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot_xacro_path:=<your_robot.urdf.xacro>
   ```

### Custom control

Edit the robot-specific config files in:
`robotnik_gazebo_ignition/config/`
You can adjust topics, frames, velocity limits, and controllers.

### Custom world

Pass a custom world file via `world_path`:

```sh
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world_path:=<your_world.sdf>
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Roadmap

* [x] Gazebo Sim support
* [ ] More worlds
* [ ] Multi-robot support

See [open issues][issues-url] for planned work and known problems.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contributing

Contributions are welcome.

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/AmazingFeature`
3. Commit: `git commit -m "Add AmazingFeature"`
4. Push: `git push origin feature/AmazingFeature`
5. Open a PR

Top contributors:

<a href="https://github.com/RobotnikAutomation/robotnik_simulation/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=RobotnikAutomation/robotnik_simulation" alt="Contributors graph" />
</a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## License

Distributed under **BSD-3**. See [`LICENSE.txt`][license-url].

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contact

Project link: [https://github.com/RobotnikAutomation/robotnik_simulation](https://github.com/RobotnikAutomation/robotnik_simulation)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Demo

[![Simulation view][product-screenshot]](https://github.com/RobotnikAutomation/robotnik_simulation)

---

<!-- LINK REFS -->

[contributors-shield]: https://img.shields.io/github/contributors/RobotnikAutomation/robotnik_simulation.svg?style=for-the-badge
[contributors-url]: https://github.com/RobotnikAutomation/robotnik_simulation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/RobotnikAutomation/robotnik_simulation.svg?style=for-the-badge
[forks-url]: https://github.com/RobotnikAutomation/robotnik_simulation/network/members
[stars-shield]: https://img.shields.io/github/stars/RobotnikAutomation/robotnik_simulation.svg?style=for-the-badge
[stars-url]: https://github.com/RobotnikAutomation/robotnik_simulation/stargazers
[issues-shield]: https://img.shields.io/github/issues/RobotnikAutomation/robotnik_simulation.svg?style=for-the-badge
[issues-url]: https://github.com/RobotnikAutomation/robotnik_simulation/issues
[license-shield]: https://img.shields.io/github/license/RobotnikAutomation/robotnik_simulation.svg?style=for-the-badge
[license-url]: https://github.com/RobotnikAutomation/robotnik_simulation/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/robotnik-automation/
[product-screenshot]: docs/assets/img/ignition_simulation_view.png
