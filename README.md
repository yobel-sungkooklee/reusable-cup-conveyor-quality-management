<!--

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                       #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) Gazebo-ROS2 Conveyor Belt Plugin. URL: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.

-->

<div id="top"></div>

<br />

<div align="center">

  <h2 align="center">Yobel_Conveyor - Reusable Cup Quality Management System</h2>

  <p align="center">
    Automated Quality Management System for Reusable Cups
    <br />
    ROS2 + Gazebo Based Simulation
    <br />
    Computer Vision Defect Detection & Automatic Sorting
  </p>
</div>

<br />

## 🏭 ABOUT

### Reusable Cup Quality Management System

This project is a comprehensive automated quality management system for reusable cups. Built upon the IFRA-Cranfield conveyor belt plugin, it implements specialized quality management features for reusable containers.

### Key Features

- **Automatic Loading**: Automatic cup generation and placement from Loader
- **QR Code Scanning**: Cup identification and history tracking
- **Computer Vision Inspection**: Automatic defect detection using cameras
- **Real-time Monitoring**: "Defect Detected / No Defects" status display
- **Automatic Sorting**: 
  - Rewash: Partially contaminated cups → Re-washing line
  - Discard: Completely damaged cups → Disposal processing

## �� INSTALLATION

This project has been developed and tested on Ubuntu 22.04 with ROS 2 Humble.

```sh
cd ~/dev_ws/src
git clone https://github.com/yourusername/Yobel_Conveyor.git
cd ~/dev_ws
colcon build
```

## 🎯 USAGE

* If you make any edits in the gazebo models, you have to do 
colcon build again!!
    ```sh
    colcon build
    ```

0. Run the below command first:

    ```sh
    source ~/dev_ws/install/setup.bash
    ```

1. Launch the ConveyorBelt Gazebo environment:

    ```sh
    ros2 launch conveyorbelt_gazebo conveyorbelt.launch.py
    ```

<!-- 2. (1) Spawn the box on top of the Belt:
  <Higher Drop>
    ```sh
    ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.0 --y -0.5 --z 1.0
    ```
  <Exact Point Drop>
    ```sh
    ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.0 --y -0.5 --z 0.76
    ``` -->

2. Activate the ConveyorBelt with the desired speed -> Value = (0,100]:

    ```sh
    ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20}"
    ```

3. Spawn the cup on top of the Belt - Enter below at 20.4XX sec (simulation time):
    ```sh
    ros2 run ros2_conveyorbelt_py cup_spawner
    ```

4. Spawn the monitor that shows defect/good cup images alternatively (when the cup passes the last LED):
    ```sh
    ros2 run ros2_conveyorbelt_py monitor_swapper
    ```

## 📁 PROJECT STRUCTURE
Yobel_Conveyor/
├── conveyorbelt_gazebo/ # Gazebo simulation environment
├── conveyorbelt_msgs/ # ROS2 message and service definitions
├── ros2_conveyorbelt/ # C++ based Gazebo plugins
└── ros2_conveyorbelt_py/ # Python utility nodes


## �� TECHNICAL FEATURES

- **ROS2 Humble** compatible
- **Gazebo** physics simulator integration
- **C++ plugins** for high-performance implementation
- **Python nodes** for easy control
- **Computer vision** based automatic defect detection
- **Real-time monitoring** and status display

## 📚 BASED ON

This project is based on the [IFRA_ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt) project.

## �� License

Apache-2.0 License (same as original IFRA project)

## 🌟 APPLICATIONS

- Smart manufacturing automation
- Reusable cup quality management
- Logistics and packaging automation
- Educational and research robot simulation
- Flexible Manufacturing System (FMS) development

## 📞 Contact

For project-related inquiries, please contact us through GitHub Issues.

---

<div align="center">
  <p><em>Building the future of reusable cup quality management</em></p>
</div>

### Original Authors
- **Mikel Bueno Viso** - Research Assistant in Intelligent Automation at Cranfield University
- **Dr. Seemal Asif** - Lecturer in Artificial Intelligence and Robotics at Cranfield University  
- **Professor Phil Webb** - Professor of Aero-Structure Design and Assembly at Cranfield University

## References

This project is based on and extends the following open-source repositories:

### Primary Reference
- **[IFRA-Cranfield/IFRA_ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt)**: The core conveyor belt simulation functionality is based on this ROS2-Gazebo plugin developed by the IFRA (Intelligent Flexible Robotics and Assembly) Group at Cranfield University. This repository provides the fundamental conveyor belt simulation capabilities that have been enhanced for reusable cup quality management.

### Original IFRA Project References
The IFRA_ConveyorBelt project itself was developed with references to:
- **[usnistgov/ARIAC](https://github.com/usnistgov/ARIAC)**: Reference for implementing a ConveyorBelt simulation with a ROS2 Plugin, adapted from the "ariac2023" branch for IFRA ROS2 Robot Simulation in Gazebo.
- **[rokokoo/gconveyor-demo](https://github.com/rokokoo/conveyor_demo)**: The CAD file (mesh) of the ConveyorBelt used in the original demo was sourced from this repository.

### Acknowledgments
Special thanks to the IFRA Group at Cranfield University for their foundational work:
- **Mikel Bueno Viso** - Research Assistant in Intelligent Automation
- **Dr. Seemal Asif** - Lecturer in Artificial Intelligence and Robotics  
- **Professor Phil Webb** - Professor of Aero-Structure Design and Assembly
