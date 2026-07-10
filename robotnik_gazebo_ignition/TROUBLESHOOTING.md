# Robotnik Gazebo Ignition Troubleshooting

This document collects the operational notes that are useful after the initial setup is complete, including installation checks, launch and control issues, GPU validation and current known limitations.

## Installation issues

### `rosdep install` fails

- Confirm that the workspace repositories were imported successfully with `vcs` before running `rosdep`.
- Make sure you are running the command from the workspace root after sourcing `/opt/ros/jazzy/setup.bash`.
- If the failure is related to missing local source packages such as `robotnik_description`, `robotnik_sensors` or `teleop_panel`, check that those repositories are present under `src/`.
- If the failure is related to `warehouse_ros_mongo`, note that Jazzy is currently being migrated to `warehouse_ros_sqlite`. The Mongo dependency is intentionally disabled in the current package manifests.

### Local Robotnik `.deb` packages are not found

- Confirm that you are running the install command from `~/ros2_ws/src/robotnik/robotnik_simulation`.
- Check that the `debs/` directory exists and contains the expected `ros-jazzy-*.deb` files.
- If the directory is missing, verify that you imported the correct repository and branch.

### `colcon build` does not find expected packages

- Confirm that the repository import step completed without errors.
- Check that the required repositories are present under `~/ros2_ws/src/robotnik/`.
- If you recently changed package dependencies or manifests, rerun `rosdep install --from-paths src --ignore-src -r -y` before rebuilding.

## Launch and spawn issues

### Gazebo launches but the robot does not appear

- Confirm that a Gazebo world is already running before launching `spawn_robot.launch.py`.
- Review the selected `robot` and `robot_model` values and make sure that combination exists in `robotnik_description`.
- If you are using `robot_xacro_path`, verify that the file exists and is readable.
- If you are spawning multiple robots, confirm that each one uses a unique `robot_id`.

### The expected ROS 2 topics are missing

- Review the selected `robot_id` and namespace in use.
- Confirm that the Gazebo bridges are active and that the robot was spawned successfully.
- If you launched only the world, remember that robot-specific topics do not exist until `spawn_robot.launch.py` completes.

### Gazebo server remains alive after closing the simulation

Gazebo Sim runs the server and the GUI as separate processes. Closing the GUI, or interrupting only part of the launch tree, may leave the Gazebo server running in the background.

When this happens, previously spawned robot models can remain loaded in the world. If those models include `gz_ros2_control`, Gazebo-side ROS 2 control nodes such as `controller_manager` and `gz_ros_control` may still appear in the ROS graph even after closing the simulation terminal.

Typical symptoms:

```bash
ros2 node list | grep -E "controller_manager|gz_ros_control"
```

still shows nodes such as:

```text
/<robot_id>/controller_manager
/<robot_id>/gz_ros_control
```

or a new empty world unexpectedly contains robots from a previous simulation run.

Check whether a Gazebo server is still running:

```bash
ps -ef | grep -E "gz sim|ign gazebo|ruby.*gz.*sim" | grep -v grep
```

You can also check whether Gazebo world services are still available:

```bash
gz service -l | grep /world/
```

If the Gazebo server is still running and you want to stop the simulation completely, use the cleanup helper:

```bash
ros2 run robotnik_gazebo_ignition stop_gazebo_simulation.sh
```

If common ROS simulation helper processes also remain, use:

```bash
ros2 run robotnik_gazebo_ignition stop_gazebo_simulation.sh --all
```

Use `--force` only if the graceful shutdown does not stop the remaining processes:

```bash
ros2 run robotnik_gazebo_ignition stop_gazebo_simulation.sh --force
```

Note that this helper is intended for development cleanup. It stops lingering Gazebo simulation processes and, with `--all`, common ROS simulation helper processes. Do not use it if you are intentionally running another Gazebo simulation in the same machine.


## RViz and MoveIt issues

### RViz opens without the expected Robotnik layout or teleoperation panel

- Confirm that RViz was started from the provided launch files instead of a manual `rviz2` invocation.
- If you use a custom RViz config, make sure it still includes the expected Robotnik displays and the `Teleop` panel.

### MoveIt does not connect or interact correctly

- Confirm that the robot was launched with `robot_id:=robot`.
- Verify that you are using a supported robot and matching `moveit_config_name`.
- If you are using a mobile manipulator, check that `arm_type` matches the robot configuration being launched.

## Control issues

### The robot does not respond to control commands

- Review the selected command topic and confirm that it matches the active `robot_id`.
- Check whether navigation, RViz teleoperation and keyboard teleoperation are publishing to the same command topic at the same time.
- If the robot was launched with a custom namespace, update the control topic accordingly.

### Teleoperation sources interfere with each other

- Avoid running multiple teleoperation or navigation sources simultaneously unless you are intentionally multiplexing commands.
- If the robot behaves unpredictably, stop all command sources and restart only the one you want to test.

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

## Known limitations

- `warehouse_ros_mongo` is intentionally disabled for Jazzy while the MoveIt warehouse integration is being migrated to `warehouse_ros_sqlite`.
- Some robots in the support table are marked as `Limited` and may still require additional validation or tuning for specific workflows.
- MoveIt support currently works correctly only with `robot_id:=robot`.

## Related

- Main package guide: [`README.md`](README.md)
- Docker workflow: [`../docker/docker.md`](../docker/docker.md)
