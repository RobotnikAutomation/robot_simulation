# Docker guide

This directory contains a unified Docker workflow for `robotnik_simulation`.

The same `docker-compose.yaml` supports both usage modes:

- run an already available image
- build the image locally when needed

## Configuration

Edit:

```bash
../env/robot.env
```

That file defines the robot, world, GUI, RViz and other bringup options used by the runtime entrypoint.

## Compose behaviour

The compose file includes both:

- `image: ${ROBOTNIK_SIMULATION_IMAGE:-robotnik_simulation:jazzy}`
- `build:`

That means:

- if you already have a suitable local image, you can use it directly
- if you want to build the image yourself, you can do it from the same compose file

## Option 1: Use an existing local image

If `robotnik_simulation:jazzy` already exists locally:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
docker compose -f docker/docker-compose.yaml up
```

## Option 2: Use a published image

If the image is published in Docker Hub or another registry:

```bash
docker pull <dockerhub_user_or_org>/robotnik_simulation:jazzy
ROBOTNIK_SIMULATION_IMAGE=<dockerhub_user_or_org>/robotnik_simulation:jazzy \
docker compose -f docker/docker-compose.yaml up
```

## Option 3: Build the image locally

If you do not have the image, or you want to generate it yourself from source:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose -f docker/docker-compose.yaml build
```

Then launch it:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose -f docker/docker-compose.yaml up
```

You can also build and launch in one step:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose -f docker/docker-compose.yaml up --build
```

## GPU mode

For NVIDIA-enabled systems:

```bash
docker compose \
  -f docker/docker-compose.yaml \
  -f docker/docker-compose.gpu.yaml \
  up
```

If you also need to build locally in GPU mode:

```bash
LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose \
  -f docker/docker-compose.yaml \
  -f docker/docker-compose.gpu.yaml \
  up --build
```

## Stop the simulation

```bash
docker compose -f docker/docker-compose.yaml down
```

## Open a shell in the running container

```bash
docker exec -it -u robot robotnik_simulation bash
```

## Notes

- Changing `env/robot.env` does not require rebuilding the image.
- Changing source code, dependencies, manifests or the `Dockerfile` does require rebuilding the image.
- `runtime-entrypoint.sh` and `Dockerfile` are only needed if you want local image build capability.
