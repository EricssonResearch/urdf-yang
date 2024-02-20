# Networked robotic system description

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Robot system model description extended with network architecture

This repository hosts the robot description files for the following components:
+ Universal Robots UR3e
+ Intel RealSense D455
+ OnRobot RG2-FT
+ Intel NUC
+ MikroTik hAP AC2
+ TP-Link TL-SG1005P PoE switch
+ SimCom sim8202g 5G modem
+ Raspberry Pi 4B

The listed model files have been extended with network topology information.

# Required packages

- Intel RealSense SDK >= 2.54
- `sudo apt-get install ros-humble-librealsense2 ros-humble-diagnostic-updater ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-joint-state-broadcaster`

# Usage/Installation

- Since these packages are wrapped to work in ROS2 environment, a ROS2 workspace needs to be created preferably in the following way:
- `mkdir -p ~/component_descr_ws && cd ~/component_descr_ws`
- Clone the repository with all the submodules into an src folder:
- `git clone --recurse-submodules https://github.com/EricssonResearch/urdf-yang.git src`
- Build the packages in the workspace:
- `colcon build`

# Testing

- Enter workspace:
- cd `~/component_descr_ws`
- Build containing packages:
- `colcon build`
- Source workspace:
- `. install/setup.bash`

## Visualizing system

- `ros2 launch system_description system_launcher.launch`

Desired output:

![](desired_output.png)

## Extracting URDF

- `ros2 param get /robot_state_publisher robot_description > robot_system_model.urdf`

# Related Publication

- M. Balogh, B. Kovács, A. Vidács, and G. Szabó, “Towards a connected robotic ecosystem,” in 2023 IEEE Conference on Standards for Communications and Networking (CSCN), 2023 [/Read Here!/](https://drive.google.com/file/d/1BM2U6pxrC8BdjCZnLkcvHCE5idf1rcdk/view?usp=sharing)

# Demonstration

[![Extending robot components with connectivity features](https://img.youtube.com/vi/lF3HMBzpvMY/0.jpg)](https://www.youtube.com/watch?v=lF3HMBzpvMY)
