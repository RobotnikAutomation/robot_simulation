# Gazebo Harmonic - ROS2 Humble

This document provides a guide to using Robotnik Simulation with:
 - Gazebo Harmonic
 - ROS2 Humble.

This combination is tested and working follow the instructions below to set up your environment.

1. Install the required packages:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

2. Install Gazebo Harmonic

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

3. Install Connect with ROS2

> **WARNING**: The package `ros-humble-ros-gzharmonic` conflicts with `ros-humble-ros-gz*`. Remove those packages before installing.

```bash
sudo apt-get remove ros-humble-ros-gz*
sudo apt-get install ros-humble-ros-gzharmonic
```

4. Clone source

```yaml
# dependencies.repos
gz_ros2_control:
  type: git
  url: https://github.com/ros-controls/gz_ros2_control.git
  version: 0.7.16
visualization_tutorials:
  type: git
  url: https://github.com/ros-visualization/visualization_tutorials.git
  version: b43b3f6a867b1d799af3edba673c56d37178e847  # ros2
robotnik_common:
  type: git
  url: https://github.com/RobotnikAutomation/robotnik_common.git
  version: 1.2.0
robotnik_description:
  type: git
  url: https://github.com/RobotnikAutomation/robotnik_description.git
  version: bacd4a4d3b2f021cdc5c4ac4f56585d15c54eaff  # Waiting for upstream release
robotnik_interfaces:
  type: git
  url: https://github.com/RobotnikAutomation/robotnik_interfaces.git
  version: 1.1.0
robotnik_sensors:
  type: git
  url: https://github.com/RobotnikAutomation/robotnik_sensors.git
  version: faaab6e1db429a6708c65d7311b148bba592fd1a  # humble-devel
robotnik_simulation:
  type: git
  url: https://github.com/RobotnikAutomation/robotnik_simulation.git
  version: jazzy-devel  # Latest version
```

```bash
vcs import --input-file dependencies.repos src
```

5. Install dependencies

```bash
sudo apt-get update
cd robotnik_simulation
sudo apt-get install -y ./debs/ros-${ROS_DISTRO}-*.deb
rosdep install --from-paths src --ignore-src -r -y
```

6. Compile workspace

```bash
export GZ_VERSION=harmonic
colcon build --symlink-install
```
