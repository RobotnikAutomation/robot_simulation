FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000

COPY dependencies/requirements/builder/packages.txt /tmp/requirements/builder-packages.txt
COPY dependencies/requirements/base/packages.txt /tmp/requirements/base-packages.txt

RUN builder_packages="$(sed "s/\${ROS_DISTRO}/${ROS_DISTRO}/g" /tmp/requirements/builder-packages.txt | grep -vE '^(#|$)')" \
    && apt-get update \
    && apt-get install -y --no-install-recommends ${builder_packages} \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN base_packages="$(sed "s/\${ROS_DISTRO}/${ROS_DISTRO}/g" /tmp/requirements/base-packages.txt | grep -vE '^(#|$)')" \
    && apt-get update \
    && apt-get install -y --no-install-recommends ${base_packages} \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi

RUN if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
    && if id -u "${USERNAME}" >/dev/null 2>&1; then \
        usermod --uid "${USER_UID}" --gid "${USER_GID}" --home "/home/${USERNAME}" --shell /bin/bash "${USERNAME}"; \
    elif getent passwd "${USER_UID}" >/dev/null; then \
        existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        usermod --login "${USERNAME}" --gid "${USER_GID}" --home "/home/${USERNAME}" --move-home --shell /bin/bash "${existing_user}"; \
    else \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi \
    && usermod -aG sudo,video,dialout "${USERNAME}" \
    && if getent group render >/dev/null; then usermod -aG render "${USERNAME}"; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p "/home/${USERNAME}/ros2_ws/src/robotnik" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

COPY docker/devel-entrypoint.sh /usr/local/bin/devel-entrypoint.sh
COPY docker/bootstrap-workspace.sh /usr/local/bin/bootstrap-workspace.sh

RUN chmod +x /usr/local/bin/devel-entrypoint.sh /usr/local/bin/bootstrap-workspace.sh

RUN printf '%s\n' \
    'if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi' \
    'if [ -f /home/robot/ros2_ws/install/setup.bash ]; then source /home/robot/ros2_ws/install/setup.bash; fi' \
    > /etc/profile.d/robotnik_ros_setup.sh \
    && chmod 0644 /etc/profile.d/robotnik_ros_setup.sh \
    && printf '%s\n' \
    'source /etc/profile.d/robotnik_ros_setup.sh' \
    >> /root/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "/home/${USERNAME}/.bashrc" \
    && echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' >> "/home/${USERNAME}/.bashrc" \
    && rosdep update

USER root

ENTRYPOINT ["/usr/local/bin/devel-entrypoint.sh"]
CMD ["sleep", "infinity"]
