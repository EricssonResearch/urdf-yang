# Topology extension for networked robotic systems


## Robot system components

| **Vendor**  | **Model** | **Status** | **Package** | **Applicable license** | **Note** |
| ----------- | --------- | ---------- | -------- | ---------------------- | -------------- |
| Universal Robots | URx/URx-e series| :white_check_mark: | [README](https://github.com/Martzi/Universal_Robots_ROS2_Description_nw_arch/blob/humble/README.md) | [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) | + control box |
| OnRobot grippers| RG2-FT | :white_check_mark: | [README](https://github.com/hsnlab-iot/onrobot_driver/blob/main/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| HEBI Robotics | X-series |  :white_check_mark: | [README](https://github.com/hsnlab-iot/hebi_yang_description/blob/master/README.md) | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)  | |
| Intel RealSense cameras| D4xx series | :white_check_mark: | [README](https://github.com/Martzi/realsense-ros-nw_arch/blob/ros2-development/README.md) | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  |
| Mobile Industrial Robots | MiR100 | :hourglass: | [README] | [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) | |
| KUKA Robotics | lrbr series | :hourglass: | [README] | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  
| KUKA Robotics | KR series | :white_check_mark: | [README](https://github.com/Martzi/kuka_robot_descriptions_nw_arch/blob/master/README.md) | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) | + control box |
| Franka Robotics | Franka robots | :hourglass: | [README] | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) | in progress... 


## Network & standalone interface components

| **Vendor**  | **Model** | **Status** | **Package** | **Applicable license** | **Note** |
| ----------- | --------- | ---------- | -------- | ---------------------- | -------------- |
| Intel NUC | NUC Mini PC | :white_check_mark: | [README](./network-components/nuc_description/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| Radio dot | general | :white_check_mark: | [README](./network-components/radiodot_description/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| Raspberry Pi | 4 model B | :white_check_mark: | [README](./network-components/raspi4b-description/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| MikroTik | hAP ac2 | :white_check_mark: | [README](./network-components/mikrotik-description/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| TP-Link | TL-SG1005P | :white_check_mark: | [README](./network-components/poesw_description/README.md) | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |

## Networked robot system example

### [There is a ROS2-based test system available, constructed from the following components:](EXAMPLE.md)
- Universal Robots UR3e
- Intel RealSense D455
- OnRobot RG2-FT
- Intel NUC
- MikroTik hAP AC2
- TP-Link TL-SG1005P PoE switch
- SimCom sim8202g 5G modem
- Raspberry Pi 4B

## Demonstration video

[![Extending robot components with connectivity features](https://img.youtube.com/vi/lF3HMBzpvMY/0.jpg)](https://www.youtube.com/watch?v=lF3HMBzpvMY)

## Related publication & citation

[M. Balogh, B. Kovács, A. Vidács, and G. Szabó, “Towards a connected robotic ecosystem,” in 2023 IEEE Conference on Standards for Communications and Networking (CSCN), 2023](https://ieeexplore.ieee.org/document/10453178)
```
@INPROCEEDINGS{10453178,
  author={Balogh, Marcell and Kovács, Bence and Vidács, Attila and Szabó, Géza},
  booktitle={2023 IEEE Conference on Standards for Communications and Networking (CSCN)}, 
  title={Towards a Connected Robotic Ecosystem}, 
  year={2023},
  volume={},
  number={},
  pages={142-147},
  doi={10.1109/CSCN60443.2023.10453178}
}
```
[G. Szabó, M. Balogh, Á. Szanyi, D. Bata, A. Vidács and I. Komlósi, "On the Connectivity Model Management of Industrial IoT Devices," 2024 IEEE 7th International Conference and Workshop Óbuda on Electrical and Power Engineering (CANDO-EPE), 2024](https://ieeexplore.ieee.org/document/10772748)
```
@INPROCEEDINGS{10772748,
  author={Szabó, Géza and Balogh, Marcell and Szanyi, Ádám and Bata, Dániel and Vidács, Attila and Komlósi, István and Pepó, Tamás},
  booktitle={2024 IEEE 7th International Conference and Workshop Óbuda on Electrical and Power Engineering (CANDO-EPE)}, 
  title={On the Connectivity Model Management of Industrial IoT Devices}, 
  year={2024},
  volume={},
  number={},
  pages={213-218},
  keywords={Visualization;Power engineering;Service robots;Network topology;Systems architecture;Fourth Industrial Revolution;Asset management;Complex systems;Industrial Internet of Things;cyber-physical systems;asset management;con-nectivity},
  doi={10.1109/CANDO-EPE65072.2024.10772748}
}
```
