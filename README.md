# Comau CRC Open ROS 2 Driver

[![DOI](https://zenodo.org/badge/1075309612.svg)](https://doi.org/10.5281/zenodo.17969100)

CRCOpen is an option available on Comau robot controller (CRC) cabinets to enable control of a robotic
system by an external Linux computer. For more detail on CRCOpen see the documentation by Comau or [click here](https://www.comau.com/en/our-offer/products-and-solutions/robotic-control-and-software/open-controller/). This driver is developed on top of CRCOpen to allow the user to be able to unlock the ROS 2 ecosystem for research and development use.

  > Note: The orl_driver binary included in this repository is provided by Comau and must be used only in compliance with an existing CRCOpen agreement.

ros2_control is the primary framework within the ROS 2 ecosystem for managing control and access to robot hardware. Modularity is achieved through separate packages for hardware interfaces, robot descriptions, and controllers. For more detail, users should familiarise themselves with the [ros2_control documentation](https://control.ros.org/rolling/index.html).

This repository contains:

__crcopen_hardware__

This contains the ROS 2 driver itself. In ros2_control parlance, this package is a [Hardware Component](https://control.ros.org/humble/doc/getting_started/getting_started.html#hardware-components) of the System type.

__comau_robots__

A directory of robot description packages for Comau robots. At the moment only the [Comau NJ130-2.6](https://www.comau.com/en/our-offer/products-and-solutions/robot-team/nj_130_2_6/) is included. 

__comau_bringup__

A ROS 2 package with useful launch files as explained in the usage section of this README.

## Installation

Install ROS 2 Rolling following the [offical instructions for Ubuntu](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html).

Clone via Git or download the ZIP archive, depending on your preference:
```bash
git clone <this-repo-url>
cd comau-crcopen-ros2-driver
``` 

Install dependencies

<details>
  <summary> Dependency installation notes </summary>

  For more information on using rosdep to manage package dependencies, see the [official documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Rosdep.html). The following uses the following parameters:

  | Parameter      | Description                                                 |
  | :--------------------- | :---------------------------------------------------------- |
  | `--from-paths`         |   Tells rosdep to scan for package.xml files under the current directory (.) alternatively can use src/  e.g. `--from-paths src`                                        |
  | `--ignore-src`     |  Skips any packages that you already have as source in this workspace, so rosdep only installs external system packages.  

  The ORL Driver debian package is provided with this code for easy installation. If already installed, you can skip this step.
</details>

```bash
sudo rosdep init # Only once
rosdep update
rosdep install --from-paths . --ignore-src -r -y

apt-get install -y ./crcopen_hardware/lib/orl_driver-*.deb
``` 

Compile and source the packages
```bash
colcon build
source install/setup.bash
```

### Using Docker (Optional)

Install [Docker for your machine](https://docs.docker.com/engine/install/).

**Note**: For real robot I/O, we recommend native Ubuntu with a realtime kernel; the container is ideal for development, fixes, and feature work.

The provided Dockerfile completes the driver installation steps automatically, ready for development. You can change the `ROS_DISTRO` build argument in the dockerfile to use a different supported distribution. The provided Docker Compose configuration will run the image and mount a volume so that edits made in the container update your local directory too.

To build and run the image in the background, then enter into a bash terminal, run
```bash
docker compose up --build -d
docker compose exec dev bash
```

To exit the container run `exit` or Ctrl-D. Then shutdown the container run
```bash
docker compose down
```

## Setup

**Setup the robot:**

Using the teach pendant, navigate to Setup->Motion->CRCOpen and select the axes to be controlled in open mode. The "External IP" must match the IP address of the computer running this driver.

**Configure this driver:**

Edit the parameters in `comau_**_description/urdf/**.ros2_control.xacro` and rebuild the packages. The IP addresses must be set correctly, and the payload parameters may optionally be set to improve the precision of the robot's internal control system. 

# Usage

> [!CAUTION]
> Operating an industrial robot can be dangerous. Users are expected to have appropriate risk assessment and mitigation in place. This ROS 2 interface is experimental and should not be used for any safety function.

This driver conforms as much as possible to the standards of [ros2_control](https://control.ros.org/rolling/index.html) so advanced users may use it in this way.

The `comau_bringup` package provides some launch files for convenience to be used as follows:

### Minimal usage

```
ros2 launch comau_bringup comau_core.launch.py
```

This starts the core functionality of the driver, including a controller manager node and joint states publisher. For example, this can be used when wanting to publish the robot state in ROS 2 while moving the robot using the teach pendant.

### Topic based control

```
ros2 launch comau_bringup comau_control.launch.py mode:=*
```

This starts the core functionality in addition to a topic-based controller of the specified mode (`position`, `velocity`, `acceleration`, `current`, `torque`). The robot can then be controlled by sending `Float64MultiArray` messages to the `/*_controller/commands` topic.

### Other ros2_control controllers

If you want a controller beyond the built-in **topic-based** ones, you can implement a custom ros2_control controller. See the [official guide](https://control.ros.org/rolling/doc/ros2_controllers/doc/writing_new_controller.html) for a step-by-step walkthrough.

You can also reuse one of the generic controllers provided by [ros2_controllers](https://github.com/ros-controls/ros2_controllers). Some of these use topics in different ways (for example, a PID controller can take a reference input from a topic), while others are action- or service-driven (e.g. trajectory controllers).

```
ros2 launch comau_bringup comau_custom_controller.launch.py controller:=* param_file:=**
```

This starts the core functionality in addition to a custom ros2_control controller. The path to a controller parameter YAML file must be provided.

## Driver Details

### Operating principals

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

### Using different robot models

Currently only the NJ-130-2.6 is supported. Other models may be added to the comau_robots folder following a similar pattern to the existing robot description packages. 

A suitable URDF will need to be created which can be based on [published CAD files](https://www.comau.com/en/our-offer/products-and-solutions/robot-team/). 

In addition, the parameters for each joint in a `*.ros2_control.xacro` will need to be populated.
 - `ruckig_max_vel` can be set based on the robot's technical specifications.

## Disclaimers

This ROS 2 driver was developed by UKAEA RACE Cybernetics engineers for internal research. While it has been tested extensively on our Comau NJ-130-2.6, it is provided as-is for research and development use only. **It is not intended for production or safety-critical applications** and may contain unforeseen bugs or limitations.

**Community-maintained**
- At the time of writing, we are actively building and testing against ROS 2 Rolling and Jazzy on Ubuntu 24.04; future support for newer ROS 2 or Ubuntu releases will depend on available resources.
- Contributions of new robot descriptions or enhancements are welcome — see CONTRIBUTING.md for details.
- At the time of writing, this driver has been verified with ORL driver 4.41.5.31647. Older versions such as 4.41.4.31545 may partially work but are known to have issues. Compatibility with other ORL driver releases is therefore not guaranteed; however, we expect this driver to work with ORL 4.41.5 and above.

> **No warranty:** This software is provided without any express or implied warranty. Use at your own risk.
