import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/spider_gait_engine.py"
with open(file_path, "r") as f:
    content = f.read()

# Revert stance parameters
content = re.sub(r'STAND_THIGH = [\d\.]+', 'STAND_THIGH = 0.6', content)
content = re.sub(r'STAND_TIBIA = \-?[\d\.]+', 'STAND_TIBIA = -0.8', content)

# Revert hardware safety clamps
content = re.sub(r'thigh = max\(0\.0, min\(1\.2, thigh\)\)', 'thigh = max(0.2, min(0.95, thigh))', content)
content = re.sub(r'tibia = max\(-1\.5, min\(-0\.4, tibia\)\)', 'tibia = max(-1.2, min(-0.4, tibia))', content)

with open(file_path, "w") as f:
    f.write(content)
