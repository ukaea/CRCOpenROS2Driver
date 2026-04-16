# ROS distribution to use (rolling | jazzy | humble, etc.)
ARG ROS_DISTRO=rolling

FROM ros:$ROS_DISTRO
# Update system
RUN apt-get update && apt-get upgrade -y

SHELL ["/bin/bash", "-c"] 

# Create a workspace and copy packages
WORKDIR /ros2_ws
COPY comau_bringup src/comau_bringup
COPY comau_robots src/comau_robots
COPY crcopen_hardware src/crcopen_hardware

# Install CRCOpen ORLDriver
RUN apt-get install -y ./src/crcopen_hardware/lib/orl_driver-*.deb

# Install dependencies
RUN rosdep update && rosdep install --from-paths . -y --ignore-src

# Build the driver
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

# Source ROS and driver packages in future shells
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# Keep the same entrypoint
COPY --chmod=755 ./entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
