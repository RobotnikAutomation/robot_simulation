Robotnik Automation Simulations
===============================

[Robotnik Automation](https://robotnik.es) is a Spanish company specialized in the development of industrial robots and automation systems. This repository contains the simulation models of the robots and the environment used in the [Robotnik Automation](https://robotnik.es) laboratories.

Short description: Robotnik Automation Simulations, for simulating the robots and the environment used in the [Robotnik Automation](https://robotnik.es) laboratories.
Available robots
----------------

- [RB-THERON](https://robotnik.eu/es/productos/robots-moviles/rb-theron/) `rb-theron`. Differential wheeled mobile robot designed for indoor applications.

- [RB-VOGUI](https://robotnik.eu/es/productos/robots-moviles/rb-vogui/) ~~`rb-vogui`~~.
- [RB-1 BASE](https://robotnik.eu/es/productos/robots-moviles/rb-1/) ~~`rb1-base`~~.
- [SUMMIT-XL STEEL](https://robotnik.eu/es/productos/robots-moviles/summit-xl-steel/) ~~`summit-xl-steel`~~.
- [SUMMIT-XL](https://robotnik.eu/es/productos/robots-moviles/summit-xl-es/) ~~`summit-xl`~~.
- [RB-CAR](https://robotnik.eu/es/productos/robots-moviles/rb-car/) ~~`rb-car`~~.

Supported simulators
--------------------

- [Gazebo Classic](https://classic.gazebosim.org/) `gazebo`.

- [Gazebo](https://gazebosim.org/) ~~`gz`~~.
- [Webots](https://cyberbotics.com/) ~~`webots`~~.
- [V-REP](https://www.coppeliarobotics.com/) ~~`vrep`~~.


Supported ROS distributions
---------------------------

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html) `humble`. Supported from May, 2022 to May, 2027.

Versions
--------

- `0.1.2` (2023-04-21).

Prerequisites
-------------

- Nvidia GPU with installed drivers. With the following command you can check if you have the correct drivers installed:

        nvidia-smi

- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker). Please follow the instructions for your distribution.

> **Note:** To ensure that all GUI applications work with dedicated GPU, you may need select it with the following command: `sudo prime-select nvidia`. To switch back to the integrated GPU, use `sudo prime-select intel`.

Usage
-----

The following docker compose file can be used to run the simulation of the RB-THERON robot in Gazebo. The `ROBOT_ID` variable is used to set the name of the robot. The `ROBOT_DESCRIPTION_FILE` variable is used to set the robot configuration. The `POS_X`, `POS_Y` and `POS_Z` variables are used to set the initial position of the robot.

```yml
version: "3.7"
services:
  rb-theron-gazebo:
    image: robotnik/robotnik-simulations:rb-theron-gazebo-humble-0.1.2
    environment:
      DISPLAY: ${DISPLAY}
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
      ROBOT_ID: robot
      ROBOT_DESCRIPTION_FILE: dual_laser.urdf.xacro
      POS_X: 0.0
      POS_Y: 0.0
      POS_Z: 0.1
    volumes:
      - type: bind
        source: /tmp/.X11-unix
        target: /tmp/.X11-unix
    runtime: nvidia
    network_mode: host
```

To run the simulation, execute the following command:

    docker-compose up

> **Note:** Images tags are composed by the following elements: `<robot>-<simulator>-<ros-distro>-<version>`. For example, the tag `rb-theron-gazebo-humble-0.1.2` means that the image is for the RB-THERON robot, it uses Gazebo as simulator and it is compatible with ROS2 Humble.

Support
-------

For issues and help troubleshooting, please contact us [support@robotnik.es](mailto:support@robotnik.es).

License
-------

As with all Docker images, these likely also contain other software which may be under other licenses (such as Bash, etc from the base distribution, along with any direct or indirect dependencies of the primary software being contained).

As for any pre-built image usage, it is the image user's responsibility to ensure that any use of this image complies with any relevant licenses for all software contained within.
