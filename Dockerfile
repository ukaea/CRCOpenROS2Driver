FROM ubuntu:24.04

# ROS distribution to use (rolling | jazzy | humble, etc.)
ARG ROS_DISTRO=rolling

# ---------- base ----------
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8
SHELL ["/bin/bash", "-c"]

# Basic tools only
RUN apt-get update -y \
 && apt-get install -y --no-install-recommends \
      locales curl gnupg2 lsb-release ca-certificates build-essential wget \
 && update-ca-certificates \
 && locale-gen en_US.UTF-8 \
 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# Pre-fetch libcap-dev over HTTPS, install via apt; fallback to dpkg + fix
RUN wget -O /tmp/libcap-dev_2.66-5ubuntu2.2_amd64.deb \
      https://security.ubuntu.com/ubuntu/pool/main/libc/libcap2/libcap-dev_2.66-5ubuntu2.2_amd64.deb \
&& apt-get update -y \
&& (apt-get install -y /tmp/libcap-dev_2.66-5ubuntu2.2_amd64.deb \
     || (dpkg -i /tmp/libcap-dev_2.66-5ubuntu2.2_amd64.deb && apt-get -f install -y)) \
&& rm -f /tmp/libcap-dev_2.66-5ubuntu2.2_amd64.deb

RUN apt-get update -y \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      python3-colcon-common-extensions python3-argcomplete \
      python3-pip python3-rosdep

# Workspace (clean, standard name)
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Bring in entrypoint + setup script (kept simple & readable)
COPY --chmod=755 ./scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY --chmod=755 ./scripts/install_dev.sh /usr/local/bin/install_dev.sh

# Keep the same entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
