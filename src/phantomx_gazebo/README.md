# Spider Robot Movement Logic & Sensor Integration

This document outlines the core logic and architecture implemented for the physical Spider Robot (Hexapod), powered by the Jetson Eagle 101, Hiwonder LSC-32, and SICK MultiScan 100 LiDAR. The simulated Gazebo dependencies have been decoupled to allow pure hardware execution.

## 1. Spider Gait Engine (`spider_gait_engine.py`)

The movement logic is controlled by a True Cartesian Inverse Kinematics (IK) engine, coupled with a robust Finite State Machine (FSM) to safely manage the physical transitions of the servos under structural load.

### Finite State Machine (FSM)
To prevent the robot from crashing into the ground or burning out the servos during deployment, the engine uses time-parameterized S-Curve (Trapezoidal Velocity) interpolation:

* **[STATE: IDLE / SITTING]**: (0.0 to 5.0 seconds). The node clamps all 18 servos to their Flat Pose (0.0 rad) to allow the servos to pre-tension and establish holding torque before lifting the chassis.
* **[STATE: LIFTING / DEPLOYING]**: (5.0 to 7.0 seconds). A geometric S-Curve dynamically calculates the Cartesian IK required to push the robot's Z-axis (`STAND_Z = -0.15m`) upwards, unfolding the legs synchronously so the feet do not slide on the ground.
* **[STATE: WALKING]**: The core walking loop is activated. The system listens to `/cmd_vel` and passes velocities through an acceleration filter (0.1) at 50Hz to mimic physical inertia and prevent sudden jerks.
* **[STATE: CONTROLLED DESCENT]**: Triggered via `rospy.on_shutdown()`. When the node is killed, it immediately locks the walking engine and runs a reverse 2.0-second S-Curve interpolation back to the Flat Pose before shutting down, ensuring the robot sits safely.

### Gait Parameters
* **Cycle Time:** 0.8 seconds (Tripod Gait)
* **Strides:** `STRIDE_X = 0.05` (Forward/Back), `STRIDE_Y = 0.02` (Strafe), `STRIDE_T = 0.04` (Turn)
* **Stance Height:** `STAND_Z = -0.15m`

## 2. IMU & Kinematic Odometry (`spider_odometry.py`)

To prevent the SLAM map from distorting during rotation and strafing, the odometry is perfectly synchronized with the hardware kinematics and the SICK industrial IMU.

* **SICK IMU Integration:** The external IMU on `/imu/data` is magnetically corrupted by the 18 high-power servos during movement. The odometry strictly ignores it and subscribes to the flawless internal IMU of the SICK LiDAR on `/multiScan/imu` (fallback to `/imu`).
* **Yaw Tracking:** Physical slipping of the legs during strafing/turning is tracked perfectly using the SICK IMU's ENU (East-North-Up) Yaw.
* **Acceleration Inertia Filter:** The odometry perfectly traces the physical startup and slowdown phases of the hexapod by mirroring the `0.1` smoothing filter of the gait engine using time-based integration (`5.0 * dt`).
* **Separated Universes:** Odometry publishes a perfectly flat `base_link` frame for 2D `slam_toolbox`, and a true tilted `base_link_3d` frame for 3D `octomap_server`.

## 3. LiDAR & SLAM Synchronization (`mapping_filter.py`)

To fix "jumping" pointclouds and SLAM TF extrapolation errors (which happen when hardware clocks drift):

* **Timestamp Synchronization:** The script intercepts the raw LaserScans from the SICK MultiScan and overwrites the hardware `header.stamp` with the exact `rospy.Time.now()` of the Jetson. This perfectly binds the laser scans to the exact millisecond of the generated odometry.
* **Continuous Mapping:** Scans are routed directly to `cloud_flat` at a continuous 15Hz to maintain a permanent lock on the walls during SLAM scan-matching.

## Main Launch Command
To run the fully integrated system (Physical Robot + SLAM + Pointclouds + RViz):
```bash
roslaunch hexapod spider_real_robot_slam.launch
```