# Robotnik Gazebo Ignition Troubleshooting

This document collects the operational notes that are useful after the initial setup is complete, including GPU checks, NVIDIA-specific launch variants and common troubleshooting cases.

## GPU verification

If you want to check whether NVIDIA is available and whether Gazebo is really using it for rendering, use this quick verification flow. The same checks can be run either on the host or inside the Docker container, depending on where you plan to execute the simulation.

### 1. Verify that the NVIDIA driver is available

Run:

```bash
nvidia-smi
```

If this command fails, NVIDIA is not available in the current environment.

### 2. Check which OpenGL renderer is being used

If `glxinfo` is not available in the current environment, install `mesa-utils` first:

```bash
sudo apt-get update
sudo apt-get install -y mesa-utils
```

Then run:

```bash
glxinfo | grep "OpenGL renderer"
```

Typical outcomes:

- `NVIDIA`: Gazebo should be using NVIDIA for OpenGL rendering.
- `Mesa Intel(...)`: rendering is going through the integrated Intel GPU.
- `llvmpipe`: rendering is falling back to software mode.

### 3. Hybrid Intel/NVIDIA systems

On hybrid Intel/NVIDIA systems, Gazebo may still render through Intel even if NVIDIA is available. In that case, you can force NVIDIA when launching `spawn_world`:

```bash
source ~/ros2_ws/install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

## Common issues

- The robot does not appear in the simulation:
  confirm that a Gazebo world is already running before launching `spawn_robot.launch.py`.
- The expected ROS 2 topics are missing:
  review the selected `robot_id`, the namespace in use and the active bridges.
- MoveIt does not connect or interact correctly:
  confirm that the robot was launched with `robot_id:=robot`.
- The robot does not respond to control commands:
  review the control topics, the selected controller setup and the active simulation state.

## Related

- Main package guide: [`README.md`](README.md)
- Docker workflow: [`../docker/docker.md`](../docker/docker.md)
