[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

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

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about">About The Project</a>
    </li>
    <li>
      <a href="#quick-start">Quick start</a>
      <ul>
        <li><a href="robotnik_gazebo_ignition/README.md">Gazebo Ignition</a></li>
      </ul>
    </li>
    <li><a href="#docker">Docker</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

---

## About
This repository provides Gazebo Sim–based environments and launch assets for simulating Robotnik platforms in ROS 2. It also points to the required Robotnik packages used across simulations.

[![Simulation view][product-screenshot]](https://github.com/RobotnikAutomation/robotnik_simulation)
![rbvogui_gif](docs/assets/img/RBVogui_Docking.gif)

## Quick start

This repository contains the following simulation packages:

* [`robotnik_gazebo_ignition`](robotnik_gazebo_ignition/README.md)


## Roadmap

* [x] Gazebo Sim support
* [ ] More worlds
* [ ] Multi-robot support

See [open issues][issues-url] for planned work and known problems.

## Contributing

Contributions are welcome.

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/jazzy/AmazingFeature`
3. Commit: `git commit -m "Add AmazingFeature"`
4. Push: `git push origin feature/AmazingFeature`
5. Open a PR and describe your changes


Special thanks to all contributors!

<a href="https://github.com/RobotnikAutomation/robotnik_simulation/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=RobotnikAutomation/robotnik_simulation" alt="Contributors graph" />
</a>

## License

Distributed under **BSD-3**. See [`LICENSE`][license-url].

## Contact

Project link: [https://github.com/RobotnikAutomation/robotnik_simulation](https://github.com/RobotnikAutomation/robotnik_simulation)

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
[license-url]: https://github.com/RobotnikAutomation/robotnik_simulation/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/robotnik-automation/
[product-screenshot]: docs/assets/img/ignition_simulation_view.png
