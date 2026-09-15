#!/bin/bash
mkdir -p /home/nammy/SD_Storage/catkin_ws/maps

echo "Saving 2D Map from SLAM Toolbox to SD Card..."
rosrun map_server map_saver -f /home/nammy/SD_Storage/catkin_ws/maps/spider_2d_map

echo "Saving 3D OctoMap Pointcloud to SD Card..."
rosrun octomap_server octomap_saver -f /home/nammy/SD_Storage/catkin_ws/maps/spider_3d_map.bt

echo "Maps saved successfully to /home/nammy/SD_Storage/catkin_ws/maps/"
