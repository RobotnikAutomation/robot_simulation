ARG base_image="robotnik/ros"
ARG ros_distro="jazzy"
ARG image_base_version="0.6.2"
ARG ros_mirror="ros.mirror.robotnik.ws"

FROM ${base_image}:${ros_distro}-builder-${image_base_version} AS builder

ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install compiled packages
RUN --mount=type=bind,\
target=/tmp/requirements.txt,\
source=dependencies/requirements/builder/packages.txt \
    true \
    && if \
        timeout 2 curl -IsS http://${ros_mirror} &>/dev/null; \
        then \
        sed -i \
            "s#packages.ros.org#${ros_mirror}#" \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && apt-fast update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

USER ${USER_NAME}

RUN --mount=type=bind,\
source=./dependencies/repos/common.repo.yml,\
target=/tmp/common.repo.yml,ro \
        vcs import \
        --input /tmp/common.repo.yml  \
        --shallow


# Generate deb packages
RUN generate_debs.sh

RUN cp /home/robot/robot_ws/src/robotnik_simulation/debs/ros-${ROS_DISTRO}-*.deb /home/robot/robot_ws/debs
WORKDIR /home/robot/robot_ws/debs
# Generate Packages.gz
RUN dpkg-scanpackages . | gzip -9c > Packages.gz


# BASE
FROM ${base_image}:${ros_distro}-base-${image_base_version} AS base

# Add Gazebo GPG key
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'\
    && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

USER root
# Install compiled packages and dependencies
RUN \
    --mount=\
type=bind,\
from=builder,\
source=/home/robot/robot_ws/debs,\
target=/tmp/debs \
    --mount=\
type=bind,\
target=/tmp/requirements.txt,\
source=dependencies/requirements/base/packages.txt \
    true \
    && if \
        timeout 2 curl -IsS http://${ros_mirror} &>/dev/null; \
        then \
        sed -i \
            "s#packages.ros.org#${ros_mirror}#" \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && echo "deb [trusted=yes] file:///tmp/debs/ ./" | tee /etc/apt/sources.list.d/debs.list \
    #&& cp /etc/apt/sources.list.d/debs.list /home/robot/robot_ws/ && cp -r /tmp/debs /home/robot/robot_ws/ \
    && apt-get update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    #&& sed -i "s#${ros_mirror}#packages.ros.org#" /etc/apt/sources.list.d/ros-latest.list \
    && dpkg -i $(find /tmp/debs -name "*.deb" | xargs) \
    #&& cp -r /tmp/debs /home/robot/robot_ws/ \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/sources.list.d/debs.list \
    && true

USER ${USER_NAME}

# The image is built to run gazebo ignition by default if no other setup is provided.
ENV STARTUP_TYPE="launch"
ENV ROS_BU_PKG="robotnik_gazebo_ignition"
ENV ROS_BU_LAUNCH="spawn_world.launch.py"

ENV QT_X11_NO_MITSHM=1
