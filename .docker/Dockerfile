# syntax=docker/dockerfile:1
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base AS roscon-workshop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-ur-simulation-gz \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      git \
      python3-pip \
      vim \
      nano \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/root/workshop_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# RosTeamWorkspace: provides the rosd/rosds/cb/cbnt/ct/ca aliases used in the workshop.
RUN git clone --depth 1 https://github.com/b-robotized/ros_team_workspace.git /opt/ros_team_workspace \
    && pip3 install -r /opt/ros_team_workspace/rtwcli/requirements.txt --break-system-packages

# The workshop material itself, so `rosds && cd roscon2026_motion_planning_workshop && git pull` works.
RUN git clone https://github.com/b-robotized/roscon2026_motion_planning_workshop.git \
      src/roscon2026_motion_planning_workshop

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "export ROS_WS=${ROS_WS}" >> ~/.bashrc \
    && echo "source /opt/ros_team_workspace/setup.bash" >> ~/.bashrc \
    && echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> ~/.bashrc
ENV TERM=xterm-color

CMD ["bash"]
