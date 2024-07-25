# Topology extension for networked robotic systems


## Robot system components

| **Vendor**  | **Model** | **Status** | **Package** | **Applicable license** | **Note** |
| ----------- | --------- | ---------- | -------- | ---------------------- | -------------- |
| Universal Robots | URx/URx-e series| :white_check_mark: | [README](./Universal_Robots_ROS2_Description_nw_arch/README.md) | [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) | + control box |
| OnRobot grippers| RG2-FT | :white_check_mark: | [README]() | [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) | |
| HEBI Robotics | X-series |  :white_check_mark: | [README](https://github.com/hsnlab-iot/hebi_yang_description/blob/master/README.md) | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)  | |
| Intel RealSense cameras| D4xx series | :white_check_mark: | [README](./realsense-ros-nw_arch/README.md) | [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  |
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
