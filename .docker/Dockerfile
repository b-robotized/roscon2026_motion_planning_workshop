# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS roscon-workshop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Same package set as ros-jazzy-desktop (the original image's base), installed
# explicitly on top of ros-base so this also builds for arm64.
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop \
      bash-completion \
      git \
      git-lfs \
      iputils-ping \
      nano \
      openssh-client \
      python3-colcon-common-extensions \
      python3-pip \
      python3-vcstool \
      sudo \
      tmux \
      trash-cli \
      tree \
      vim \
      wget \
    && rm -rf /var/lib/apt/lists/*

RUN git config --system --add safe.directory "*"

# Allow system-wide pip installs (PEP 668) without --break-system-packages everywhere.
RUN python_version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')") \
    && externally_managed_file_path="/usr/lib/python${python_version}/EXTERNALLY-MANAGED" \
    && if [ -f "$externally_managed_file_path" ]; then mv "$externally_managed_file_path" "$externally_managed_file_path.old"; fi
RUN pip3 install -U -I pre-commit

# Non-root user (the original was built via RTW's rocker-based --standalone docker
# workflow, which injects a non-root user; this reproduces that without needing rocker).
# Ubuntu's base image already ships a "ubuntu" user (uid/gid 1000, in the sudo group) --
# it just needs NOPASSWD sudo.
ARG USERNAME=ubuntu
RUN echo "${USERNAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}
ENV ROS_WS=/home/${USERNAME}/roscon2026_motion_planning_ws
# pip installs user-site (non-root) console scripts here, including rtw itself.
ENV PATH=/home/${USERNAME}/.local/bin:${PATH}

RUN git clone -b master https://github.com/b-robotized/ros_team_workspace.git /home/${USERNAME}/ros_team_workspace \
    && cd /home/${USERNAME}/ros_team_workspace/rtwcli \
    && pip3 install -r requirements.txt

RUN mkdir -p ${ROS_WS}/src/ur \
    && git clone https://github.com/b-robotized/roscon2026_motion_planning_workshop.git \
         ${ROS_WS}/src/roscon2026_motion_planning_workshop \
    && git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git \
         ${ROS_WS}/src/ur/Universal_Robots_ROS2_Description \
    && git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git \
         ${ROS_WS}/src/ur/Universal_Robots_ROS2_Driver \
    && git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
         ${ROS_WS}/src/ur/Universal_Robots_ROS2_GZ_Simulation

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && sudo apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --ignore-src -y --rosdistro ${ROS_DISTRO} --from-paths ${ROS_WS}/src \
    && sudo rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd ${ROS_WS} \
    && MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN mkdir -p /home/${USERNAME}/.ros_team_workspace \
    && cat > /home/${USERNAME}/.ros_team_workspace/workspaces.yaml <<EOF
workspaces:
  roscon2026_motion_planning_ws:
    base_ws: null
    distro: ${ROS_DISTRO}
    docker_container_name: null
    docker_devices: []
    docker_tag: null
    env_vars: {}
    standalone: false
    ws_docker_support: false
    ws_folder: ${ROS_WS}
    ws_name: roscon2026_motion_planning_ws
EOF

RUN echo '. /home/'"${USERNAME}"'/ros_team_workspace/setup.bash' >> ~/.bashrc \
    && echo 'if [ -f ~/.ros_team_ws_rc ]; then . ~/.ros_team_ws_rc; fi' >> ~/.bashrc \
    && echo '. /home/'"${USERNAME}"'/ros_team_workspace/scripts/configuration/terminal_coloring.bash' >> ~/.bashrc \
    && echo 'export __GLX_VENDOR_LIBRARY_NAME=mesa' >> ~/.bashrc \
    && echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc \
    && echo 'rtw workspace use roscon2026_motion_planning_ws' >> ~/.bashrc
ENV TERM=xterm-color

CMD ["bash"]
