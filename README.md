# Comau CRC Open ROS2 Driver

CRCOpen is an option available on Comau robot controller (CRC) cabinets to enable control of a robotic
system by an external Linux computer. For more detail on CRCOpen see the documentation by Comau or [click here](https://www.comau.com/en/our-offer/products-and-solutions/robotic-control-and-software/open-controller/). This driver is developed on top of CRCOpen to allow the user to be able to unlock the ROS 2 ecosystem for research and development use.

  > Note: The orl_driver binary included in this repository is provided by Comau and must be used only in compliance with an existing CRCOpen agreement.

ros2_control is the primary framework within the ROS 2 ecosystem for managing control and access to robot hardware. Modularity is achieved through separate packages for hardware interfaces, robot descriptions, and controllers. For more detail, users should familiarise themselves with the [ros2_control documentation](https://control.ros.org/rolling/index.html)

This repository contains:

__crcopen_hardware__

This contains the ROS 2 driver itself. In ros2_control parlance, this package is a [Hardware Component](https://control.ros.org/rolling/doc/getting_started/getting_started.html#hardware-components) of the System type.

__comau_robots__

A directory of robot description packages for Comau robots. At the moment only the [Comau NJ130-2.6](https://www.comau.com/en/our-offer/products-and-solutions/robot-team/nj-130-2-6) is included. 

__comau_bringup__

A ROS 2 package with useful launch files as explained in the usage section of this readme.

# Release Status

**Overall pipeline status**
<!-- Overall pipeline status -->
[![CI](https://github.com/ukaea/CRCOpenROS2Driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ukaea/CRCOpenROS2Driver/actions/workflows/ci.yml)


# Getting Started


## Installation

Follow the steps below on an **Ubuntu 24.04** terminal (preferably running the Real-time Ubuntu kernel with the PREEMPT_RT patch for improved determinism and low latency). This ROS 2 control driver officially supports Rolling on the main branch, Jazzy on a separate actively maintained branch, and includes an unmaintained Humble branch for legacy use.

<details>
  <summary> How to install ROS 2 Rolling </summary>

  For a complete, step-by-step walkthrough, please see the official ROS 2 Rolling installation guide:  

  https://docs.ros.org/en/rolling/Installation.html

  #### System update & essentials

  Update your system and install essential packages  

  ```bash
  sudo apt-get update && sudo apt-get upgrade -y
  ```

  ```bash
  sudo apt-get install -y --no-install-recommends \
  locales curl gnupg2 lsb-release software-properties-common \
  build-essential ca-certificates
  ```

  #### Configure locale

  Configure your locale to UTF-8

  ```bash
  locale-gen en_US en_US.UTF-8
  ```

  ```bash
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  ```

  ```bash
  export LANG=en_US.UTF-8
  ```

  #### Add the ROS 2 apt repository

  ```bash
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

  ```bash
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
  ```

  ```bash
  sudo apt-get update
  ```

  #### Install ROS 2 base and dev tools
  
  Replace `ROS_DISTRO` with `rolling`, `jazzy` as required:
  ```bash
  export ROS_DISTRO=rolling 
  ``` 

  ```bash
  sudo apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-ros-base \
  python3-colcon-common-extensions \
  python3-argcomplete python3-pip python3-rosdep
  ```

  ```bash
  source /opt/ros/${ROS_DISTRO}/setup.bash
  ```
</details>


<details>
  <summary> Using Docker </summary>

  **Prereqs:** Docker and Docker compose installed. The provided Dockerfile sets up Ubuntu 24.04 with ROS 2 (default: `${ROS_DISTRO}=jazzy`).  
  You can change the `ROS_DISTRO` build argument in the dockerfile to use a different supported distribution.


  1) **Clone & enter the repo**
  ```bash
  git clone <this-repo-url>

  cd comau-crcopen-ros2-driver
  ```

  2) **Start the dev container**

  This builds the image and starts the container. On first start it will also:
  - install the ORL driver if the .deb is present in the repo,
  - run rosdep for dependencies.
  ```bash
  docker compose --env-file .env.dev up --build -d
  ```

  3) **Enter the container**
  
  ```bash
  docker compose exec dev bash
  ```

  4) **Source ROS and build**
  ```bash
  cd /ros2_ws
  source /opt/ros/${ROS_DISTRO}/setup.bash
  colcon build
  source install/setup.bash
  ```

  **Note**: 
  - For real robot I/O, we recommend native Ubuntu with a realtime kernel; the container is ideal for development, fixes, and feature work.
  - To reopen the same container later: 'docker start -ai comau_dev'
  - The ORL driver .deb is installed by install_docker.sh. You must have it at crcopen_hardware/lib/.
  
</details>

<details>
  <summary>How to enable Real-time Ubuntu</summary>

  We recommend reviewing the official Ubuntu documentation for detailed instructions on enabling Real-time Ubuntu:  
  - [Enable Real-time Ubuntu](https://documentation.ubuntu.com/real-time/latest/how-to/enable-real-time-ubuntu/)  
  - [About Real-time Ubuntu](https://ubuntu.com/real-time)

</details>


### Install Driver dependencies

**Clone this repository**

You can either clone via Git or download the ZIP archive, depending on your preference:
```bash
git clone <this-repo-url>
  ``` 

```bash
cd comau-crcopen-ros2-driver
  ``` 

Install all driver dependencies automatically using the provided script:

```bash
chmod +x install.sh

./install.sh
  ``` 

<details>
  <summary> Manual Installation </summary>

  If you prefer to install dependencies manually or need more control over the installation process, follow these steps:

  **Use rosdep to fetch all ROS runtime dependencies**

  ```bash
  # (Once only)
  rosdep init

  rosdep update
  # Install dependencies declared in package.xml
  rosdep install --from-paths . --ignore-src -r -y
  ```

  | Parameter      | Description                                                 |
  | :--------------------- | :---------------------------------------------------------- |
  | --from-paths         |   Tells rosdep to scan for package.xml files under the current directory (.) alternatively can use src/  e.g. --from-paths src                                        |
  | --ignore-src     |  Skips any packages that you already have as source in this workspace, so rosdep only installs external system packages.  

  **Install the ORL driver, we assume the ORL driver .deb binary file is located at crcopen_hardware/lib/orl_driver-*.deb**

  ```bash
  sudo apt install -y crcopen_hardware/lib/orl_driver-*.deb
  ```

  > Note: The path crcopen_hardware/lib/orl_driver-*.deb is relative to the root of your cloned repository. Replace it with the actual location (absolute path) of the ORL driver .deb on your system (e.g. `~/ros2_ws/src/comau-crcopen-ros2-driver/crcopen_hardware/lib/orl_driver-*.**.*.*****-Linux.deb`).

</details>


### Build the Driver

Return to the workspace root
```bash
# For example
cd ~/ros2_ws
  ``` 

Compile with colcon
```bash
source "/opt/ros/{ROS_DISTRO}/setup.bash"
colcon build
  ``` 
Source   the install setup
```bash
source install/setup.bash
  ```

The driver is now built and ready to use!

## Setup

**Setup the robot:**

Using the teach pendant, navigate to Setup->Motion->CRCOpen and select the axes to be controlled in open mode. The "External IP" must match the IP address of the computer running this driver.

**Configure this driver:**

Edit the paramters in `comau_**_description/urdf/**.ros2_control.xacro`. The IP addresses must be set correctly, and the payload parameters may optionally be set to improve the precision of the robot's internal control system.

## Usage Overview

> [!CAUTION]
> Operating an industrial robot can be dangerous. Users are expected to have appropriate risk assessment and mitigation in place. This ROS2 interface is experimental and should not be used for any safety function.

This driver conforms as much as possible to the standards of [ros2_control](https://control.ros.org/rolling/index.html) so advanced users may use it in this way.

The `comau_bringup` package provides some launch files for convenience to be used as follows:

### Minimal usage

`ros2 launch comau_bringup comau_core.launch.py`

This starts the core functionality of the driver, including a controller manager node and joint states publisher. For example, this can be used when wanting to publish the robot state in ROS 2 while moving the robot using the teach pendant.

### Topic based control

`ros2 launch comau_bringup comau_control.launch.py mode:=*`

This starts the core functionality in addition to a topic-based controller of the specified mode (`position`, `velocity`, `acceleration`, `current`, `torque`). The robot can then be controlled by sending `Float64MultiArray` messages to the `/*_controller/commands` topic.

### Other ros2_control controllers

If you want a controller beyond the built-in **topic-based** ones, you can implement a custom ros2_control controller. See the [official guide](https://control.ros.org/rolling/doc/ros2_controllers/doc/writing_new_controller.html) for a step-by-step walkthrough.

You can also reuse one of the generic controllers provided by [ros2_controllers](https://github.com/ros-controls/ros2_controllers). Some of these use topics in different ways (for example, a PID controller can take a reference input from a topic), while others are action- or service-driven (e.g. trajectory controllers).

`ros2 launch comau_bringup comau_custom_controller.launch.py controller:=* param_file:=**`

This starts the core functionality in addition to a custom ros2_control controller. The path to a controller parameter YAML file must be provided.

# Driver Details

## Operating principals

The hardware will refuse to go into an `active` state unless the teach pendant is in the AUTO TP mode. In this case, it will fall back to `inactive` and continue publishing live joint data to `/joint_states`.

Joint data will only be published for joints in **open mode**. Additionally, any command interface claims made to joints not in open mode will be refused automatically.

## Joint States

The `/joint_states` topic publishes the measured position (rad), velocity (rad/s) and effort (ampere peak) of each joint.

The `/dynamic_joint_states` topic additionally includes acceleration (rad/s^2) which is not measured but an indication of the internal interpolator when using position, velocity or acceleration control modes; and torque (Nm) which is calculated using the scaling factor `vr_TorqConst` from the measured current.

## Control modes

Allowed command interface combinations:
- "position"
    - Send position [rad] commands at any interval. Commands are internally interpolated at 400us using ruckig.
- "position_direct"
    - Send position [rad] commands directly without interpolation. Requires controller to update at a high rate.
- "velocity"
    - Send velocity [rad/s] commands at any interval. Target positions sent to CRC are internally predicted given the velocity command.
- "velocity" and "position_direct"
    - Send velocity [rad/s] commands and CRC target positions [rad] directly. Requires controller to update at a high rate.
- "acceleration"
    - Send acceleration [rad/s<sup>2</sup>] commands at any interval. Target velocities and positions are internally interpolated. This interface uses the velocity mode of the robot.
- "current"
    - Send current [A] commands at any interval. Target positions sent to CRC are internally predicted given the previous velocity.
- "current" and "position_direct"
    - Send current [A] commands and CRC target positions [rad] directly. Requires controller to update at a high rate.
- "torque"
    - Send torque [Nm] commands at any interval. Torque values are internally converted to current [A] using the vr_TorqConst factor in the xacro then handled as "current".
- "torque" and "position_direct"
    - Send torque [Nm] and CRC target positions [rad] directly. Torque values are internally converted to current [A] using the vr_TorqConst factor in the xacro then handled as "current".

Position, velocity and acceleration can have unclaimed joints because they can be held constant. Current and torque need active control of all axes with open mode enabled on the teach pendant.

## Using different robot models

Currently only the NJ-130-2.6 is supported. Other models may be added to the comau_robots folder following a similar pattern to the existing robot description packages. 

A suitable URDF will need to be created which can be based on [published CAD files](https://www.comau.com/en/our-offer/products-and-solutions/robot-team/). 

In addition, the parameters for each joint in a `*.ros2_control.xacro` will need to be populated.
 - `ruckig_max_vel` can be set based on the robot's technical specifications.

# Contributing

PRs are welcome to improve the driver. Additions of robot descriptions, where these have been verified on a real system, are particularly encouraged.

For more details regarding contributing please read CONTRIBUTING.md carefully.

<!-- ### MoveIt

1. Open CoppeliaSim Scene or attach Dynamixel U2D2 USB Cable

2. Launch ROS2 Control, spawning a joint trajectory controller

   `ros2 launch src/launch/macro.launch.py robot:=viperx hardware:=coppeliasim controller:=joint_trajectory`

3. Launch moveit MoveGroup and RViz

   `ros2 launch viperx_moveit_config move_group.launch.py`

   `ros2 launch viperx_moveit_config moveit_rviz.launch.py`

The above is made easier by the single launch file 

`ros2 launch src/launch/moveit_macro.launch.py robot:=viperx hardware:=coppeliasim` --> 

## Goals, Disclaimer & Support

This ROS2 driver was developed by UKAEA RACE Cybernetics engineers for internal research. While it has been tested extensively on our Comau NJ-130-2.6, it is provided as-is for research and development use only. **It is not intended for production or safety-critical applications** and may contain unforeseen bugs or limitations.

**Community-maintained**
- At the time of this writing, we are actively building and testing against ROS 2 Rolling and Jazzy on Ubuntu 24.04; future support for newer ROS 2 or Ubuntu releases will depend on available resources.
- Contributions of new robot descriptions or enhancements are welcome—see CONTRIBUTING.md for details.
- At the time of this writing, this driver has been verified with ORL driver 4.41.5.31647. Older versions such as 4.41.4.31545 may partially work but are known to have issues. Compatibility with other ORL driver releases is therefore not guaranteed; however, we expect this driver to work with ORL 4.41.5 and above.

> **No warranty:** This software is provided without any express or implied warranty. Use at your own risk.
