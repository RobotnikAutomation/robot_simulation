# Docker development guide

This document describes the development-oriented Docker workflow for `robotnik_simulation` on `jazzy-devel`.

The Docker environment prepares the workspace automatically inside the container by:

- importing the external repositories required by `robotnik_simulation`
- installing the Robotnik-specific `.deb` packages shipped in this repository
- resolving the remaining dependencies with `rosdep`

The workspace build remains a manual step inside the container.

The container is intentionally configured to re-check and re-apply the local `.deb` installation and `rosdep install` steps every time it starts. This is done on purpose to keep the development environment flexible instead of assuming a fixed preconfigured runtime state.

## Prerequisites

Before using the Docker workflow, make sure you have:

- Docker installed
- permission to run Docker commands with `sudo`
- access to an X11 session if you want to launch Gazebo or RViz with GUI

Start from the repository root:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
```

Run every `LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) sudo docker compose ...` command from this repository root.

## Installation and first setup

Use this section the first time you create the Docker environment, or whenever you need to rebuild the image from source.

### 1. Build and start the development container

Use the Docker Compose file under `docker/` to build the image and start the container:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) sudo docker compose -f docker/docker-compose.yaml up --build
```

This command:

- builds the `robotnik_simulation:jazzy-devel` image
- creates the persistent workspace volumes
- starts the `robotnik_simulation_devel` container
- runs the development entrypoint, which prepares the workspace before leaving the container ready for interactive use

> **Important**: the container is not considered ready until the entrypoint finishes importing repositories, installing local `.deb` packages and running `rosdep`.

> **Terminal behaviour**: this command stays attached to the container output. Once the startup sequence has finished, press `Ctrl+C` if you want to recover that host terminal and continue working from other terminals with `docker exec`.

### 2. Open a shell inside the container

Once the container is running, open an interactive shell as the `robot` user:

```bash
sudo docker exec -it -u robot robotnik_simulation_devel bash
```

Use the `robot` user for development and builds to avoid permission issues in the workspace.

### 3. Build the workspace inside the container

The Docker entrypoint prepares the environment, but the workspace build is still manual:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Daily usage

Once the image has already been built and the workspace has been compiled at least once, normal development sessions do not require `up --build` again.

### 1. Start the existing container again if it was stopped

Use this command only if the container already exists and was previously stopped:

```bash
sudo docker start -a robotnik_simulation_devel
```

This command keeps the current terminal attached to the container output while the development entrypoint runs again.

If the container is already running, you do not need to call `docker start`.

Once the startup sequence is complete, press `Ctrl+C` if you want to reuse that host terminal. The container will keep running in the background.

### 2. Open terminal 1 inside the container

Open a new terminal on the host and enter the running container:

```bash
sudo docker exec -it -u robot robotnik_simulation_devel bash
```

When you want to leave a shell inside the container and return to the host terminal, run:

```bash
exit
```

### 3. Launch Gazebo world in terminal 1

After the workspace is built and sourced, you can launch the simulation normally:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

If the host machine has NVIDIA graphics and you want to force Gazebo to use it from inside the Docker container, launch the world with:

```bash
cd ~/ros2_ws
source install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

### 4. Open terminal 2 and spawn the robot

`spawn_robot.launch.py` requires Gazebo to already be running, so launch it from a second shell inside the same container.

Open another terminal on the host and enter the container again:

```bash
sudo docker exec -it -u robot robotnik_simulation_devel bash
```

Then run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py robot:=rbwatcher
```

### 5. Optional: use the integrated bringup flow instead

Instead of using terminal 1 for `spawn_world` and terminal 2 for `spawn_robot`, you can launch the integrated bringup flow from a single shell inside the container:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robotnik_simulation_bringup bringup_complete.launch.py robot_model:=rbsummit use_gui:=true use_rviz:=false
```

For more bringup examples and parameters, see [`../common/robotnik_simulation_bringup/README.md`](../common/robotnik_simulation_bringup/README.md).

### 6. Stop the container but keep it for later reuse

Use `stop` when you want to end the current session but keep the container available so it can be started again later with `docker start`:

```bash
sudo docker stop robotnik_simulation_devel
```

### 7. Remove the Compose container

Use `down` when you want Docker Compose to stop and remove the current container instance while keeping the image available:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) sudo docker compose -f docker/docker-compose.yaml down
```

After `down`, the next session should start again with `docker compose ... up`, not with `docker start`.

## Optional: GPU mode

If the host has NVIDIA support configured for Docker, you can enable GPU access by adding the GPU override file:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) sudo docker compose \
  -f docker/docker-compose.yaml \
  -f docker/docker-compose.gpu.yaml \
  up --build
```

When using the GPU-enabled Docker mode on hybrid Intel/NVIDIA systems, Gazebo may still need the NVIDIA offload prefixes at launch time:

```bash
cd ~/ros2_ws
source install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=empty
```

For a complete validation flow of driver detection, Docker GPU passthrough and OpenGL renderer selection, see [`../robotnik_gazebo_ignition/README.md#gpu-verification-guide`](../robotnik_gazebo_ignition/README.md#gpu-verification-guide).

To stop and remove that GPU-enabled Compose container, use the same pair of files:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) sudo docker compose \
  -f docker/docker-compose.yaml \
  -f docker/docker-compose.gpu.yaml \
  down
```

## Cleanup and rebuild

Use this section only when you want to remove the current Docker state and start again from a clean setup.

### 1. Remove the container manually

If you need to force removal of the existing container:

```bash
sudo docker rm -f robotnik_simulation_devel
```

### 2. Remove the generated image

If you need to rebuild the image from scratch, remove it first:

```bash
sudo docker rmi robotnik_simulation:jazzy-devel
```

### 3. Remove the persistent workspace volumes

If you want a completely clean Docker workspace state, remove the persistent volumes:

```bash
sudo docker volume rm \
  robotnik_simulation_ws_workspace_src \
  robotnik_simulation_ws_workspace_build \
  robotnik_simulation_ws_workspace_install \
  robotnik_simulation_ws_workspace_log
```

## Notes

- If you modify `docker/Dockerfile`, `docker/devel-entrypoint.sh` or `docker/bootstrap-workspace.sh`, rebuild the image with `docker compose ... up --build`.
- If the container starts again after `docker stop`, the development entrypoint will re-check the workspace, local `.deb` installation and `rosdep` state before leaving the container ready.
- If you only modify the repository source code, the changes are visible inside the container through the bind mount and no image rebuild is required.
- The Docker workflow is intended for development on `jazzy-devel`; it is not a release image flow.
- Work is still in progress on a separate release-oriented Docker image that can be distributed through Docker Hub and used as a direct simulation launcher, configured through environment variables without requiring the development workflow described above.
