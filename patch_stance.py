import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/spider_gait_engine.py"
with open(file_path, "r") as f:
    content = f.read()

# Update stance parameters
content = re.sub(r'STAND_THIGH = [\d\.]+', 'STAND_THIGH = 0.785', content)
content = re.sub(r'STAND_TIBIA = \-?[\d\.]+', 'STAND_TIBIA = -0.785', content)

# Update hardware safety clamps to accommodate the new stance and lifting paths
content = re.sub(r'thigh = max\(0\.2, min\(0\.95, thigh\)\)', 'thigh = max(0.0, min(1.2, thigh))', content)
content = re.sub(r'tibia = max\(-1\.2, min\(-0\.4, tibia\)\)', 'tibia = max(-1.5, min(-0.4, tibia))', content)

with open(file_path, "w") as f:
    f.write(content)
