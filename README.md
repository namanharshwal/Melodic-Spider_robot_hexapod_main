# Spider Robot SLAM & Simulation Workspace

Welcome to the catkin workspace for the custom 6-legged (hexapod) spider robot project. This workspace integrates 3D LiDAR SLAM, Gazebo simulation, physical servo control, and autonomous gait generation on a Jetson Eagle 101 single-board computer.

---

## 1. Hardware Architecture & Wiring

The robot's physical layout is composed of the following components:

```mermaid
graph TD
    LiPo[12V 5400mAh LiPo Battery] --> Jetson[Jetson Eagle 101 Brain]
    LiPo --> LSC32[Hiwonder LSC-32 v1.3 Servo Controller]
    Jetson -- Ethernet -- USB-to-TTL Cable --> LSC32
    LSC32 --> Servos[18x Smart Serial Servos]
    Jetson -- Ethernet -- Ethernet Cable --> LiDAR[SICK MultiScan 100 3D LiDAR]
```

### Components Details
* **Main Brain:** **Jetson Eagle 101** (4GB RAM, 128GB SD Card Storage).
* **Power Supply:** **12V 5400mAh Lithium Polymer (LiPo) Battery** supplying power to both the Jetson board and the servo controller board.
* **Actuation Controller:** **Hiwonder LSC-32 v1.3** 32-Channel Servo Controller.
  * Connected to the Jetson via a **USB-to-TTL Serial Cable** on port `/dev/ttyUSB0` at **9600 Baud**.
  * Directly drives 18 smart serial servos (3 joints per leg: Coxa, Femur/Thigh, Tibia across 6 legs).
* **LiDAR Sensor:** **SICK MultiScan 100** (16-layer 3D LiDAR).
  * **LiDAR IP Address:** `192.168.0.1`
  * **Jetson IP Address:** `192.168.0.10`
  * Connected via Ethernet to the Jetson Eagle 101.

---

## 2. Workspace Structure & Packages

The workspace is named `catkin_ws` and is located at `/home/nammy/SD_Storage/catkin_ws`.

```
catkin_ws/
├── src/
│   ├── phantomx_description/      # URDF and 3D visual models of the hexapod
│   ├── phantomx_control/          # Gazebo Joint Effort Controllers config & PIDs
│   ├── phantomx_gazebo/           # Gazebo worlds, walker nodes, and hardware serial bridges
│   ├── hexapod/                   # Launch files for mapping and main bringing up
│   ├── sick_scan_xd/              # Official driver for SICK MultiScan 100 LiDAR
│   └── Hopper_ROS/                # Alternative hexapod code and utilities
└── .agents/
    └── rules/
        └── spider_robot_guidelines.md  # Always-on AI system memory & settings
```

### Key Source Files & Descriptions
* **[lsc32_hardware_bridge.py](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/lsc32_hardware_bridge.py)**
  Translates ROS `JointState` messages (representing joint angles in radians) into byte command packets sent over serial to the Hiwonder LSC-32 servo board. It maps the 18 joints dynamically and bounds them to prevent damage.
* **[walker.py](file:///home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/walker.py)**
  The gait generation engine. It calculates leg kinematics (tripod gait based on sine wave phase functions) and subscribes to `/cmd_vel` to move the robot in simulation and hardware.
* **[spider_mapping.launch](file:///home/nammy/SD_Storage/catkin_ws/src/hexapod/launch/spider_mapping.launch)**
  Launches:
  1. The SICK MultiScan LiDAR driver pointing to `192.168.0.1` and sending data to `192.168.0.10`.
  2. A static transform publisher (`tf`) linking the LiDAR frame (`cloud`) to the robot chassis (`base_link`) with a 15cm Z-axis offset.
  3. `slam_toolbox` for online asynchronous 2D mapping.
  4. `octomap_server` to generate 3D volumetric maps from the point cloud topic `/cloud_unstructured_fullframe`.

---

## 3. Quick Start & Execution Commands

Ensure your ROS environment is sourced before running:
```bash
source devel/setup.bash
```

### A. Run Simulation Only (Headless)
Brings up the Gazebo environment and the gait engine:
```bash
roslaunch phantomx_gazebo phantomx_gazebo.launch
```

### B. Run Simulation + Real Robot Control + Keyboard Teleop
Brings up the Gazebo gait engine, launches the serial bridge to the physical LSC-32 board on `/dev/ttyUSB0`, and opens a keyboard controller:
```bash
roslaunch phantomx_gazebo phantomx_real_robot.launch
```

### C. Run Real Robot Control with Joint State GUI & RViz Visualization
Allows you to slide sliders in a GUI to manually control the angles of each of the 18 servos on the real hardware while visualizing the model in RViz:
```bash
roslaunch phantomx_gazebo phantom_real_robot.launch
```

### D. Launch 3D LiDAR Mapping & SLAM
Starts the LiDAR connection, static transforms, `slam_toolbox` mapping, and 3D Octomap builder:
```bash
roslaunch hexapod spider_mapping.launch
```

### E. Launch Everything (Real Robot + SLAM + RViz + Teleop) with a Single Command
Brings up the Gazebo gait engine, launches the physical servo bridge to the LSC-32 controller, initiates the SICK MultiScan 100 LiDAR driver, establishes base transforms, starts 2D SLAM and 3D Octomaps, opens RViz, and enables keyboard teleop control in the terminal:
```bash
roslaunch hexapod spider_real_robot_slam.launch
```

---

## 4. Persistent AI Conversation Context

To resume pair-programming with the AI assistant on the Jetson Eagle 101, use the following commands. The workspace is configured with an always-on rule (`.agents/rules/spider_robot_guidelines.md`) so that the AI remembers the entire hardware/software setup.

* **Resume the current chat session:**
  ```bash
  agy --conversation dce98fed-1524-488a-95f7-2396e430fbc7
  ```
* **Alternatively, resume the last session automatically:**
  ```bash
  agy -c
  ```
