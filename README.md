# Jaw PixArm

6 DoF robotic arm project using **ROS 2 Jazzy Jalisco** and **Dynamixel hardware**. 
This repository contains the source code, control configuration, and launch files needed to build and run the system.

<p align="center">
  <img src="images/jaw.png" alt="Jaw PixArm" width="400"/>
</p>



---

## 📂 Repository Structure
- `src/` – main source code (nodes, launch files, packages)
- `ros2_control/` – ROS 2 control configuration for hardware/simulation
- `dynamixel_hardware/` – drivers and hardware interfaces for Dynamixel servos
- `printFiles_stl/` – 3D print files .stl

---

## 🚀 Prerequisites

- **OS:** Ubuntu 24.04 (Noble Numbat)
- **Middleware:** [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- **Build tools:** `colcon`

Install common dependencies to control with the ps4 controller:

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y \
  python3-colcon-common-extensions \
  ros-jazzy-ros2-control \
  ros-jazzy-dynamixel-sdk
  ros-jazzy-joy
```

---
# Start the nodes and move the arm
You will have to open three terminal windows.
The joy library allows our program to communicate with a PS4 or other game controllers. It maps the physical buttons, triggers, and analog sticks into software-readable events. In this case, subscribe to joy_node to get all the information needed to create a controll system for the robotic arm.
In the first terminal window write:
```bash
colcon build --packages-select giou_ps4
source install/setup.bash
ros2 run joy joy_node
```

In the second terminal window run the ps4_control node:
```bash
colcon build --packages-select giou_ps4
source install/setup.bash
ros2 run giou_ps4 ps4_control.py
```

In the third terminal window start the last node move_dynamixel:
```bash
colcon build --packages-select giou_ps4
source install/setup.bash
ros2 run giou_ps4 move_dynamixel.py
```

Now will see a lot of informations about the status of the servomotors and the ros2 nodes, to see if everything started in order and properly and for debug.

---

Design inspired from [Alexander Koch](https://github.com/AlexanderKoch-Koch/low_cost_robot) and fully adapted for dynamixel xl430-w250-t servomotors.


---

## License
This project is licensed under the MIT License. See the LICENSE file for details
