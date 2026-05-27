# Docker runtime guide

This directory contains the end-user Docker assets for `robotnik_simulation`.

The user is expected to already have access to a built image, either:

- pulled from Docker Hub
- or loaded from a `.tar` file

The user does not need the image build sources, `Dockerfile`, or runtime entrypoint script.

## What The User Needs

- Docker installed
- permission to run Docker commands
- the runtime image
- this `docker-compose.yaml`
- `../env/robot.env`
- optionally `docker-compose.gpu.yaml` for NVIDIA systems

## Configure The Simulation

Edit:

```bash
../env/robot.env
```

That file controls the robot, model, GUI, RViz, world and other bringup options.

## Launch With A Local Image

If the image already exists locally:

```bash
cd ~/ros2_ws/src/robotnik/robotnik_simulation
docker compose -f docker/docker-compose.yaml up
```

By default, the compose file uses:

```bash
robotnik_simulation:jazzy
```

## Launch With A Published Image

You can override the image name at runtime:

```bash
ROBOTNIK_SIMULATION_IMAGE=<dockerhub_user_or_org>/robotnik_simulation:jazzy \
docker compose -f docker/docker-compose.yaml up
```

## Pull Before Launching

If the image is published in Docker Hub:

```bash
docker pull <dockerhub_user_or_org>/robotnik_simulation:jazzy
ROBOTNIK_SIMULATION_IMAGE=<dockerhub_user_or_org>/robotnik_simulation:jazzy \
docker compose -f docker/docker-compose.yaml up
```

## Load From A Tarball

If you received the image as a tar file:

```bash
docker load -i robotnik_simulation_jazzy.tar
docker compose -f docker/docker-compose.yaml up
```

## GPU Mode

For NVIDIA-enabled systems:

```bash
ROBOTNIK_SIMULATION_IMAGE=<dockerhub_user_or_org>/robotnik_simulation:jazzy \
docker compose \
  -f docker/docker-compose.yaml \
  -f docker/docker-compose.gpu.yaml \
  up
```

## Stop The Simulation

To stop and remove the Compose container:

```bash
docker compose -f docker/docker-compose.yaml down
```

## Open A Shell In The Running Container

```bash
docker exec -it -u robot robotnik_simulation bash
```

## Notes

- This directory is runtime-only.
- Image build and publish workflows live in `../robotnik_docker_simulation/`.
- Changing `env/robot.env` does not require rebuilding the image.
