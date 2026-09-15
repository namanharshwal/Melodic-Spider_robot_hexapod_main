---
trigger: always_on
description: "Persistent guidelines, hardware specs, network configurations, serial settings, and history memory for the Spider Robot SLAM workspace."
---

# Spider Robot Project Guidelines & Persistent Memory

This rule is automatically loaded for the `/home/nammy/SD_Storage/catkin_ws` workspace. It stores the exact hardware specifications, software architecture, launch configurations, and project history to maintain seamless context across all development sessions.

## 1. Hardware Specifications
* **Main Brain:** Jetson Eagle 101
  * **RAM:** 4GB
  * **Storage:** 128GB SD Card
* **Battery:** 12V 5400 mAh Lithium Polymer (LiPo) battery
* **Servo Controller:** Hiwonder LSC-32 v1.3
  * **Communication:** Serial communication via USB-to-TTL cable to Jetson
  * **Default Port:** `/dev/ttyUSB0`
  * **Baud Rate:** 9600
  * **Servos:** 18 smart serial servos controlling 6 legs (LF, LM, LR, RF, RM, RR). Each leg has Coxa, Femur (Thigh), and Tibia joints.
* **3D LiDAR:** SICK MultiScan 100 (16-layer 3D LiDAR)
  * **LiDAR IP Address:** `192.168.0.1`
  * **Jetson (Receiver) IP Address:** `192.168.0.10`

## 2. Workspace Software Architecture
The workspace name is `catkin_ws` located at `/home/nammy/SD_Storage/catkin_ws`. Key packages include:
* [phantomx_gazebo](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo): Contains simulation drivers and physical hardware bridges.
  * [lsc32_hardware_bridge.py](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/lsc32_hardware_bridge.py): Interacts with the Hiwonder LSC-32 v1.3 controller, converting joint states in radians to LSC position packets.
  * [walker.py](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/walker.py): Generates walking gaits (sinusoidal tripod/gait functions) and listens on `/phantomx/cmd_vel`.
* [hexapod](file:///home/nammy/SD_Storage/catkin_ws/src/hexapod): Defines launches and SLAM configurations.
  * [spider_mapping.launch](file:///home/nammy/SD_Storage/catkin_ws/src/hexapod/launch/spider_mapping.launch): Brings up the SICK MultiScan 100 LiDAR driver (`sick_scan_xd`), sets up a static transform (`base_link` -> `cloud`), starts `slam_toolbox` for 2D mapping, and starts `octomap_server` for 3D volumetric mapping.
* [phantomx_description](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_description): Contains URDF/xacro descriptions of the robot model.
* [sick_scan_xd](file:///home/nammy/SD_Storage/catkin_ws/src/sick_scan_xd): Driver package for the SICK MultiScan 100.

## 3. Operations & Quick Command Reference
* **Start Simulation & Gait Engine (Headless):**
  ```bash
  roslaunch phantomx_gazebo phantomx_gazebo.launch
  ```
* **Bringup Real Robot Servos + Keyboard Teleop + Simulation:**
  ```bash
  roslaunch phantomx_gazebo phantomx_real_robot.launch
  ```
* **Bringup Real Robot (Custom Joint GUI control + Teleop + RViz):**
  ```bash
  roslaunch phantomx_gazebo phantom_real_robot.launch
  ```
* **Bringup LiDAR SLAM Mapping (Real Hardware):**
  ```bash
  roslaunch hexapod spider_mapping.launch
  ```
* **Bringup Real Robot + LiDAR SLAM Mapping + RViz + Teleop (Single Unified Command):**
  ```bash
  roslaunch hexapod spider_real_robot_slam.launch
  ```

## 4. Conversation History & Resuming Chat
* **Session ID:** `dce98fed-1524-488a-95f7-2396e430fbc7`
* **How to Resume:**
  To continue this conversation history at any time on the Jetson, open your terminal and run:
  ```bash
  agy --conversation dce98fed-1524-488a-95f7-2396e430fbc7
  ```
  Or to launch the last conversation:
  ```bash
  agy -c
  ```

## 5. Critical Hardware & Code Behaviors Discovered
* **I2C Potentiometer Feedback:** 
  * The physical feedback is read using ADS1115 ADC modules (address `0x48` on bus `1`).
  * `real_feedback_publisher.py` handles the physical readings and merges them with simulated fallbacks.
  * `calibrate_sensors.py` is available for viewing live raw voltages for calibration.
* **LSC-32 Beeping/Alarms:**
  * **Continuous Beep (Low Voltage):** Triggered if the 12V LiPo battery is disconnected or the power switch is OFF while the USB-to-TTL provides 5V. **Never connect the Red VCC wire from the USB-TTL cable to the board.**
  * **Short Beeps (Corrupted Packet Alarm):** Triggered by invalid serial formatting. 
* **Python 2 vs 3 Serial Bug:** 
  * ROS Melodic runs in a Python 2.7 environment. When writing serial packets for the LSC-32, the code must use `bytearray([...])` rather than `bytes([...])`. In Python 2, `bytes` is an alias for `str` and converts list integers into ASCII string representations (e.g. `"[85, 85, 8...]"`). This caused the LSC-32 to read corrupted characters, fail to move, and alarm. All bridge scripts now enforce `#!/usr/bin/env python` and use `bytearray` for cross-compatibility.
