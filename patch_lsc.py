import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/lsc32_hardware_bridge.py"
with open(file_path, "r") as f:
    content = f.read()

replacement = """
def rad_to_lsc(angle_rad, joint_name):
    \"\"\"
    Convert joint angle (radians) to LSC-32 PWM microseconds (500-2500).
    \"\"\"
    angle_rad = max(-1.2, min(1.2, angle_rad))
    
    # 180-degree servos (j_thigh_rr, j_thigh_lr): 90 degrees = 1.57079 rad = 1000 PWM
    # 270-degree servos (All others): 135 degrees = 2.35619 rad = 1000 PWM
    if joint_name in ['j_thigh_rr', 'j_thigh_lr']:
        lsc_pos = 1500 + int((angle_rad / 1.57079) * 1000)
    else:
        lsc_pos = 1500 + int((angle_rad / 2.35619) * 1000)
"""

content = re.sub(r'def rad_to_lsc\(angle_rad, joint_name\):[\s\S]*?lsc_pos = 1500 \+ int\(\(angle_rad \/ 2\.35619\) \* 1000\)', replacement.strip(), content)

with open(file_path, "w") as f:
    f.write(content)
