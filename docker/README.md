# Docker guide

This directory contains the Docker files used to run `robotnik_simulation`.

The simulation configuration is defined in:

```bash
../env/robot.env
```

Edit that file to choose the robot, world, GUI, RViz and other bringup options before launching the container.

## Prerequisites

- Docker must be installed.
- Your user should have permission to use Docker.
- If your user is not in the Docker group, run the commands below with `sudo`.

## 1. Run the published image

Use this option when the image is already published in Docker Hub or another registry.

From the repository root:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
```

Pull the image:

```bash
docker pull robotnik/simulation-gz:jazzy
```

Launch the simulation:

```bash
ROBOTNIK_SIMULATION_IMAGE=robotnik/simulation-gz:jazzy \
docker compose -f docker/docker-compose.yaml up
```

If you prefer, you can also export `ROBOTNIK_SIMULATION_IMAGE` in the shell before launching.

To stop the published-image run started with `docker compose ... up`, press `Ctrl+C` once to stop the simulation gracefully. If the container does not exit cleanly, press `Ctrl+C` again to force it to stop.

## 2. Build a new image locally

Use this option when you want to generate the image from source with the current contents of the repository.

From the repository root:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
```

Build the image:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose -f docker/docker-compose.yaml build
```

Launch the simulation with the locally built image:

```bash
docker compose -f docker/docker-compose.yaml up
```

You can also build and launch in a single step:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose -f docker/docker-compose.yaml up --build
```

## 3. Daily workflow

Once the container is running, keep the first terminal attached to:

```bash
docker compose -f docker/docker-compose.yaml up
```

That terminal shows the bringup logs and lets you stop the session with `Ctrl+C`.

If you want to work inside the running container, open a second terminal and enter it as the `robot` user:

```bash
docker exec -it -u robot robotnik_simulation bash
```

Inside that shell you can inspect topics, launch extra nodes or interact with the simulated robot using normal ROS 2 commands such as:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /clock
```

If you prefer to leave the simulation running in the background, start it detached:

```bash
docker compose -f docker/docker-compose.yaml up -d
```

Then attach a shell with `docker exec` whenever you need to work inside the container.

To stop the running container without removing it:

```bash
docker stop robotnik_simulation
```

To start that same stopped container again and reattach to its logs:

```bash
docker start -a robotnik_simulation
```

To stop and remove the Compose container completely:

```bash
docker compose -f docker/docker-compose.yaml down
```

To remove the local image:

```bash
docker rmi robotnik/simulation-gz:jazzy
```

If you are using a published image name instead of the default local tag, remove that specific image tag instead.

## Notes

- Changing `../env/robot.env` does not require rebuilding the image.
- Changing source code, dependencies, manifests or `docker/Dockerfile` does require rebuilding the image.
- For NVIDIA systems, use `docker/docker-compose.gpu.yaml` together with the main compose file.
