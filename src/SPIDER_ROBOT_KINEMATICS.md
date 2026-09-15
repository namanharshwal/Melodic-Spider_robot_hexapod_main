# Spider Robot Hexapod: Kinematics & Hardware Architecture

This document serves as the master reference for the mathematically perfect movement engine driving the PhantomX Hexapod. It is designed to be accessible to beginners learning robotics, while providing the rigorous mathematical deep-dives required by professional engineers.

---

## 1. Beginner's Guide: How Does a Spider Robot Walk?

Walking is essentially controlled falling. To move without tipping over, a hexapod (6-legged robot) must keep its center of gravity inside a polygon formed by the feet currently touching the ground.

### The "Wave Gait" (Pentapod Gait)
Our robot uses a **6-Phase Wave Gait**. Unlike a "Tripod Gait" (where 3 legs lift at once), the Wave Gait only lifts **one leg at a time**. 
* **Support Phase (5 legs):** The leg pushes firmly against the ground to propel the body forward.
* **Swing Phase (1 leg):** The leg lifts into the air and swings forward to its next footprint.

By having 5 legs on the ground at all times, the robot enjoys maximum stability, which is critical when carrying heavy payloads like the SICK MultiScan 100 LiDAR and Jetson Eagle 101 computer.

### The "S-Curve" Smooth Start
Real servos are physically powerful. If we instantly commanded the robot to stand up, it would violently jerk, potentially stripping the servo gears or flipping the robot. 
To solve this, our engine implements a **5-second zero-initialization** (all servos hold perfectly flat) followed by a **2-second S-Curve Ramp** using a Hermite interpolation (`p^2 * (3 - 2p)`). This gently lifts the robot to its exact standing posture.

---

## 2. Professional Deep Dive: True 3D Cartesian Inverse Kinematics (IK)

The most naive way to program a robot is to guess the joint angles. However, guessing angles causes the feet to drag in an arc across the floor, fighting against each other and wasting battery.

Our engine uses **True 3D Cartesian Inverse Kinematics**. We calculate exactly where the foot should be in 3D space ($X, Y, Z$ coordinates in millimeters) and use trigonometry to solve for the exact joint angles required to reach that point.

### Step A: Global Twist to Local Footprint
When the user sends a ROS `Twist` command via teleop ($V_x$ forward, $V_y$ strafe, $V_z$ turn), we convert this into a 3D displacement vector for each foot.

Because the legs are mounted in a radial pattern around the body, we use a radial rotation matrix (`LEG_THETA`) to map the global robot intent into the local frame of the leg:

```python
# Y-axis points FORWARD (90 degrees). X-axis points RIGHT (0 degrees).
dx_global = sweep * -Vy * STRIDE_Y
dy_global = sweep * Vx * STRIDE_X

# Project global vector onto the leg's local coordinate system using Rotation Matrix
local_y_outward = dx_global * cos(theta) + dy_global * sin(theta)
local_x_sideways = dx_global * -sin(theta) + dy_global * cos(theta)
```

**Perfect Tangential Turning:**
To turn in place, a robot should not act like a tank. The front and rear legs must sweep in a circular arc. We achieve mathematically perfect turning by adding the rotational velocity directly to the **tangential axis** (`local_x_sideways`) of every leg, independent of its mounting angle.

### Step B: The Trigonometric Solver
Given the target $(X, Y, Z)$ footprint, we solve the joints:
1. **Coxa (Yaw):** `c1 = math.atan2(x, y)`
2. **Femur (Pitch):** Using the Law of Cosines on the triangle formed by the Femur length ($L_{THIGH}$), Tibia length ($L_{TIBIA}$), and the distance to the target footprint ($L$).
3. **Tibia (Pitch):** Using the Law of Cosines for the inner knee angle.

### Step C: The "Delta IK" Architecture (The Secret Sauce)
The user wanted a highly specific, custom-tuned standing posture (`Thigh=0.8, Tibia=-0.8`). If we hardcoded this into the IK engine's geometry, the Cartesian math would deform. 

Instead, we invented **Delta IK**:
1. Run the IK solver for the *Current* walking footprint.
2. Run the IK solver for the *Baseline* default footprint.
3. Subtract the two to isolate the pure **Movement Delta**.
4. Add the Movement Delta directly to the user's custom standing posture.

This provides the mathematical perfection of True Cartesian IK while allowing infinite, non-destructive cosmetic customization of the robot's stance.

---

## 3. Simulation to Real-Hardware Bridge

Bridging Gazebo simulation logic to real-world servos introduces massive data pipeline challenges.

### Overcoming UART Saturation (The "Dancing" Bug)
The robot uses 18 Hiwonder serial servos daisy-chained to an LSC-32 v1.3 controller. The controller communicates with the Jetson via a USB-to-TTL serial cable at `9600 baud`.

* **The Math:** 9600 baud allows ~960 bytes per second. A full 18-servo command packet is 61 bytes. Therefore, transmission takes **63.5 milliseconds**.
* **The Bug:** Originally, the ROS bridge was blasting commands at 80ms intervals. Python's overhead plus the 63.5ms transmission time caused the serial buffer to overflow. The LSC-32 received corrupted packets, causing the robot to violently twitch, "dance in place", and sound its alarm buzzer.
* **The Fix:** The hardware bridge (`lsc32_hardware_bridge.py`) implements a strict **100ms (10Hz) rate limit**. The internal packet `move_time` is perfectly synchronized to `100ms`, creating flawless, buttery-smooth movement without a single dropped packet.

### Python 2.7 Bytearray Serialization
ROS Melodic runs on Python 2.7. A critical bug was discovered where `bytes([...])` was translating integer lists into ASCII characters instead of raw hex bytes. The bridge now strictly enforces `bytearray([...])` to ensure binary-perfect serial communication to the LSC-32.

### ADS1115 Encoder Data Feedback
To close the loop between the simulation and the real world, the robot is equipped with ADS1115 Analog-to-Digital Converters (ADCs) on the I2C bus (`bus 1`, address `0x48`). 
* These ADCs read the internal potentiometers of the RDS3115 servos.
* The script `real_feedback_publisher.py` converts these raw voltages into radian joint states.
* If a leg hits an obstacle in the real world, the ROS TF tree dynamically updates to reflect the physical blockage, allowing the SLAM mapping algorithms (slam_toolbox/octomap) to correct for the terrain.

---

## 4. Final System Statistics & Parameters

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Gait Type** | 6-Phase Wave Gait | Pentapod (1 leg up, 5 down) |
| **Cycle Time** | 0.8 Seconds | Time to complete one full step |
| **Duty Cycle** | 16.6% (1/6) | Percentage of time a leg spends in the air |
| **L_COXA** | 54 mm | Physical length of Coxa joint |
| **L_THIGH** | 66 mm | Physical length of Femur joint |
| **L_TIBIA** | 120 mm | Physical length of Tibia joint |
| **Max Stride X** | 5.0 cm | Forward/Backward step length |
| **Max Stride Y** | 2.0 cm | Left/Right strafe step length |
| **Turn Lift Safety** | 1.5 cm | Dynamic Z-lift applied to prevent foot drag during rotations |
| **UART Baud** | 9600 bps | Hardcoded limit of LSC-32 v1.3 |
| **Bridge Frequency**| 10 Hz | Rate-limited to prevent saturation |
