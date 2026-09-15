import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/servo_feedback_reader.py"
with open(file_path, "r") as f:
    content = f.read()

replacement = """
# Locked Calibration for the 270-Degree Encoder Servo (Servo 0)
CALIBRATION['j_tibia_lr'] = {'v_min': 0.081, 'v_max': 3.006, 'theta_min': -2.35619, 'theta_max': 2.35619}
# Locked Calibration for the 270-Degree Encoder Servo (Servo 1)
CALIBRATION['j_thigh_lr'] = {'v_min': 0.084, 'v_max': 2.960, 'theta_min': -1.57079, 'theta_max': 1.57079} # UPDATED TO 180 DEG
# Locked Calibration for the 270-Degree Encoder Servo (Servo 2)
CALIBRATION['j_c1_lr'] = {'v_min': 0.083, 'v_max': 3.013, 'theta_min': -2.35619, 'theta_max': 2.35619}

# Update the replaced RR thigh to 180 degrees
CALIBRATION['j_thigh_rr']['theta_min'] = -1.57079
CALIBRATION['j_thigh_rr']['theta_max'] = 1.57079
"""

content = re.sub(r'# Locked Calibration for the 270-Degree Encoder Servo \(Servo 0\).*?CALIBRATION\[\'j_c1_lr\'\] = \{.*?\}', replacement.strip(), content, flags=re.DOTALL)

with open(file_path, "w") as f:
    f.write(content)
