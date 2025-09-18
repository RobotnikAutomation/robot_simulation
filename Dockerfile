ARG base_image="robotnik/ros"
ARG ros_distro="jazzy"
ARG image_base_version="0.6.2"

FROM ${base_image}:${ros_distro}-builder-${image_base_version} AS builder

ARG ros_distro

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

USER root

# Install compiled packages and dependencies
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list

RUN --mount=type=bind,\
target=/tmp/requirements.txt,\
source=dependencies/requirements/builder/packages.txt \
    apt-fast update \
    && apt-get remove -y ros-${ROS_DISTRO}-ros-gz* \
    && apt-get upgrade -y \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

USER ${USER_NAME}

RUN --mount=type=bind,\
source=./robotnik_simulation.${ros_distro}.repos,\
target=/tmp/common.repo.yml,ro \
    vcs import \
        --input /tmp/common.repo.yml  \
        --shallow src

RUN wget -O ${USER_HOME}/.ros/gazebo_rosdep.yaml \
    https://raw.githubusercontent.com/osrf/osrf-rosdep/refs/heads/master/gz/gazebo.yaml \
    && echo "yaml file://${USER_HOME}/.ros/gazebo_rosdep.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/50-gazebo-latest.list

# remove   <depend>ros_gz*</depend> exec_depend, depends and build_depend from package.xml files
RUN find src/ -type f -name 'package.xml' -exec sed -i 's/<depend>ros_gz.*<\/depend>//g; s/<exec_depend>ros_gz.*<\/exec_depend>//g; s/<build_depend>ros_gz.*<\/build_depend>//g' {} +

# Generate deb packages
RUN generate_debs.sh

RUN cp /home/robot/robot_ws/src/robotnik/robotnik_simulation/debs/ros-${ROS_DISTRO}-*.deb /home/robot/robot_ws/debs
WORKDIR /home/robot/robot_ws/debs
# Generate Packages.gz
RUN dpkg-scanpackages . | gzip -9c > Packages.gz


# BASE
FROM ${base_image}:${ros_distro}-base-${image_base_version} AS base

USER root

# Add Gazebo GPG key
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list

ENV GZ_VERSION=harmonic

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
    && echo "deb [trusted=yes] file:///tmp/debs/ ./" | tee /etc/apt/sources.list.d/debs.list \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get remove -y ros-${ROS_DISTRO}-ros-gz* \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
        /tmp/debs/ros-${ROS_DISTRO}-*.deb \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/sources.list.d/debs.list \
    && true

RUN apt-get update && apt-get upgrade -y \
    && apt-get clean -y \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

USER ${USER_NAME}

# # # The image is built to run gazebo ignition by default if no other setup is provided.
# # ENV STARTUP_TYPE="launch"
# # ENV ROS_BU_PKG="robotnik_gazebo_ignition"
# # ENV ROS_BU_LAUNCH="spawn_world.launch.py"

# # ENV QT_X11_NO_MITSHM=1
