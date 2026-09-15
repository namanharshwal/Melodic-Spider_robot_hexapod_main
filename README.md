# Spider Robot Quick Start Guide

Welcome to the Spider Robot repository. This is a quick reference for the essential launch commands. For the complete, detailed 1000+ line mathematical breakdown, history, and architectural guide, please see `Spider_Robot_Research_And_Setup.md` on the Desktop.

## Workspace Build & Setup

Before running any ROS nodes, ensure you source the workspace. If you modify any source code or are setting up the workspace for the first time, use these commands to compile:

```bash
# Source the workspace environment
source ~/SD_Storage/catkin_ws/devel/setup.bash

# To build or clean the workspace (only needed when code is changed):
cd ~/SD_Storage/catkin_ws
catkin_make clean
catkin_make
source ~/SD_Storage/catkin_ws/devel/setup.bash
```

---

## Quick Commands

### 1. Manual Drive & Map Building
Use this to manually drive the robot with a keyboard and build a map.
```bash
roslaunch hexapod spider_real_robot_slam.launch
```
*(In a separate terminal, run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`)*

### 2. Saving the Maps
Run these in a separate terminal while the mapping launch is active.
```bash
# Save 2D Map
rosrun map_server map_saver -f ~/SD_Storage/catkin_ws/src/maps/lab_505_marik_center

# Save 3D Pointcloud Octomap
rosrun octomap_server octomap_saver -f ~/SD_Storage/catkin_ws/src/pointcloud/lab_505_marik_center.bt
```

### 3. Autonomous Waypoint Navigation
Use this to load your saved map and click destinations in RViz.
```bash
roslaunch hexapod spider_navigation.launch
```
*(Use "2D Pose Estimate" then "2D Nav Goal" in RViz)*

### 4. Autonomous Frontier Mapping
Use this to let the robot automatically explore and map a brand new room.
```bash
roslaunch hexapod spider_explore.launch
```
