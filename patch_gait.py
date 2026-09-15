import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/spider_gait_engine.py"
with open(file_path, "r") as f:
    content = f.read()

# 1. Implement Tripod Gait
replacement1 = """
PHASE_OFFSETS = {
    'lf': 0.0,
    'rm': 0.0,
    'lr': 0.0,
    'rf': 0.5,
    'lm': 0.5,
    'rr': 0.5
}
"""
content = re.sub(r'PHASE_OFFSETS = \{[^\}]+\}', replacement1.strip(), content)

# 2. Fix the Kinematic logic and Tripod Duty Cycle
replacement2 = """
    def calculate_leg_ik(self, leg, cycle):
        offset = PHASE_OFFSETS[leg]
        local_cycle = (cycle + offset) % 1.0
        
        # Tripod gait perfectly lifts 3 legs at a time (50% swing, 50% support)
        duty_cycle = 0.5
        
        if local_cycle < duty_cycle:
            sub_phase = local_cycle / duty_cycle
            sweep = -math.cos(sub_phase * math.pi)
            lift = math.sin(sub_phase * math.pi) * SWING_HEIGHT
        else:
            sub_phase = (local_cycle - duty_cycle) / (1.0 - duty_cycle)
            sweep = math.cos(sub_phase * math.pi)
            lift = 0.0
            
        c1 = STAND_C1
        thigh = STAND_THIGH
        tibia = STAND_TIBIA
        
        # LIFT LOGIC: Tibia must curl INWARD (decrease) to properly clear the floor!
        thigh -= lift
        tibia -= lift * 0.8
        
        # The configuration that worked perfectly for Forward/Backward!
        # Middle legs (lm, rm) are physically inverted on the chassis!
        c1_mult = {
            'lf': 1.0, 'lm': -1.0, 'lr': 1.0,
            'rf': 1.0, 'rm': -1.0, 'rr': 1.0
        }
        c1 += sweep * self.vx * STRIDE_X * c1_mult[leg]
        
        is_left = 'l' in leg
        
        # STRAFING LOGIC
        strafe_mult = -1.0 if is_left else 1.0
        ext = sweep * self.vy * STRIDE_Y * strafe_mult
        thigh += ext
        
        # TURNING LOGIC (Correctly inverted to match physical turning physics!)
        turn_mult = -1.0 if is_left else 1.0
        c1 += sweep * self.vt * STRIDE_T * turn_mult * c1_mult[leg]
"""

content = re.sub(r'    def calculate_leg_ik\(self, leg, cycle\):[\s\S]*?c1 \+= sweep \* self\.vt \* STRIDE_T', replacement2.strip(), content)

with open(file_path, "w") as f:
    f.write(content)
