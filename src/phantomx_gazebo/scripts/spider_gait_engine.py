#!/usr/bin/env python
"""
Spider Gait Engine - FINAL RESTORATION OF AUG 25 TRUE CARTESIAN IK
This is the mathematically perfect Wave Gait + 3D Cartesian IK Engine from August 25,
combined with the safe 5-second zero-initialization startup sequence.
"""

import rospy
import math
from threading import Thread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# =========================================================================
# KINEMATICS CONFIGURATION (Meters)
# =========================================================================
L_COXA = 0.054
L_THIGH = 0.066
L_TIBIA = 0.120

# Footprint Default Position (Relative to Coxa Joint)
STAND_Y = 0.12   # Distance outwards from the body
STAND_Z = 0.12   # Distance downwards from the body

# Gait parameters (Meters)
SWING_HEIGHT = 0.04  # 4cm lift height
STRIDE_X     = 0.05  # 5cm forward/backward step
STRIDE_Y     = 0.02  # 2cm strafe left/right step
STRIDE_T     = 0.04  # 4cm turning offset
CYCLE_TIME   = 0.8   # Seconds per full step cycle

JOINTS = {
    'lf': ['j_c1_lf', 'j_thigh_lf', 'j_tibia_lf'],
    'lm': ['j_c1_lm', 'j_thigh_lm', 'j_tibia_lm'],
    'lr': ['j_c1_lr', 'j_thigh_lr', 'j_tibia_lr'],
    'rf': ['j_c1_rf', 'j_thigh_rf', 'j_tibia_rf'],
    'rm': ['j_c1_rm', 'j_thigh_rm', 'j_tibia_rm'],
    'rr': ['j_c1_rr', 'j_thigh_rr', 'j_tibia_rr']
}

ALL_JOINTS = []
for leg in JOINTS:
    ALL_JOINTS.extend(JOINTS[leg])

LEG_THETA = {
    'rf': math.radians(45),
    'rm': math.radians(0),
    'rr': math.radians(-45),
    'lf': math.radians(135),
    'lm': math.radians(180),
    'lr': math.radians(225)
}

WAVE_OFFSETS = {
    'lf': 0.0,
    'rm': 0.166,
    'lr': 0.333,
    'rf': 0.5,
    'lm': 0.666,
    'rr': 0.833
}
DUTY_CYCLE = 1.0 / 6.0

class SpiderGaitEngine:
    def __init__(self):
        rospy.init_node('spider_gait_engine')
        
        self.pub = rospy.Publisher('/phantomx/gui_commands', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vt = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vt = 0.0
        
        self.current_angles = {}
        for name in ALL_JOINTS:
            self.current_angles[name] = 0.0
            
        self.t = 0.0
        self.is_sitting = False
        
        rospy.on_shutdown(self.sit_down)
        
        self.thread = Thread(target=self.loop)
        self.thread.start()
        
        rospy.loginfo("Spider Gait Engine Initialized: TRUE CARTESIAN IK with FSM SIT/STAND.")

    def sit_down(self):
        rospy.loginfo("[FSM] STATE: CONTROLLED DESCENT - Sitting down safely...")
        self.is_sitting = True
        rate = rospy.Rate(50)
        
        start_angles = self.current_angles.copy()
        
        # 2-second controlled descent using S-Curve (Trapezoidal Velocity Profile)
        duration = 2.0
        steps = int(duration * 50)
        
        for i in range(steps):
            progress = float(i) / steps
            # Use the existing quintic/cubic smooth ramp for zero-acceleration start/stop
            scalar = self.get_smooth_ramp(progress)
            
            for name in ALL_JOINTS:
                # Interpolate from current standing angles down to Flat Pose (0.0)
                self.current_angles[name] = start_angles[name] * (1.0 - scalar)
                
            self.clamp_and_publish()
            
            try:
                rate.sleep()
            except Exception:
                pass
                
        rospy.loginfo("[FSM] STATE: REST / COLLAPSED - Robot safely on ground.")

    def cmd_cb(self, msg):
        # Clamp inputs to [-1.0, 1.0] to prevent physical over-extension of the IK legs!
        self.target_vx = max(-1.0, min(1.0, msg.linear.x))
        self.target_vy = max(-1.0, min(1.0, msg.linear.y))
        self.target_vt = max(-1.5, min(1.5, msg.angular.z))

    def calculate_leg_ik(self, leg, cycle):
        offset = WAVE_OFFSETS[leg]
        local_cycle = (cycle - offset) % 1.0
        
        if local_cycle < DUTY_CYCLE:
            is_swing = True
            phase = local_cycle / DUTY_CYCLE
        else:
            is_swing = False
            phase = (local_cycle - DUTY_CYCLE) / (1.0 - DUTY_CYCLE)
            
        if is_swing:
            # Cosine curve for smooth acceleration/deceleration (Ease-in/Ease-out)
            sweep = -math.cos(phase * math.pi)
            lift = math.sin(phase * math.pi) * SWING_HEIGHT
        else:
            sweep = math.cos(phase * math.pi)
            lift = 0.0
            
        # Global footprint displacement in Body Frame
        # Swapped: Math's X-axis is RIGHT (theta=0), Math's Y-axis is FORWARD (theta=90)
        # So ROS vx (Forward) -> dy
        # ROS vy (Left) -> -dx (Right)
        dx = sweep * -self.vy * STRIDE_Y
        dy = sweep * self.vx * STRIDE_X
        
        # Project global (dx, dy) onto the leg's local coordinate system
        theta = LEG_THETA[leg]
        
        # Local Y is OUTWARD. Local X is SIDEWAYS.
        local_y_offset = dx * math.cos(theta) + dy * math.sin(theta)
        local_x_offset = dx * (-math.sin(theta)) + dy * math.cos(theta)
        
        # Turn adds a purely tangential component to the local X (sideways) axis
        local_x_offset += sweep * self.vt * STRIDE_T
        
        # Final Cartesian coordinates for the IK solver
        x = local_x_offset
        y = STAND_Y + local_y_offset
        z = STAND_Z - lift

        # Fix for falling down during turn: lift leg slightly during turn support
        if not is_swing:
            z -= abs(self.vt) * abs(sweep) * 0.015  # 1.5cm lift to prevent dragging

        # True Cartesian Inverse Kinematics solver for CURRENT position.
        def solve_ik(x_in, y_in, z_in):
            d_in = math.sqrt(x_in**2 + y_in**2) - L_COXA
            L_in = math.sqrt(d_in**2 + z_in**2)
            max_L = L_THIGH + L_TIBIA - 0.001
            L_in = min(L_in, max_L)
            
            alpha_in = math.atan2(z_in, d_in)
            
            val2 = (L_THIGH**2 + L_in**2 - L_TIBIA**2) / (2 * L_THIGH * L_in)
            beta_in = math.acos(max(-1.0, min(1.0, val2)))
            thigh_in = alpha_in - beta_in
            
            val = (L_THIGH**2 + L_TIBIA**2 - L_in**2) / (2 * L_THIGH * L_TIBIA)
            gamma_in = math.acos(max(-1.0, min(1.0, val)))
            tibia_in = -(math.pi - gamma_in)
            
            c1_in = math.atan2(x_in, y_in)
            return c1_in, thigh_in, tibia_in

        # 1. Get the raw IK angles for the current footprint
        c1_ik, thigh_ik, tibia_ik = solve_ik(x, y, z)
        
        # 2. Get the raw IK angles for the default baseline footprint
        c1_base, thigh_base, tibia_base = solve_ik(0.0, STAND_Y, STAND_Z)
        
        # 3. Calculate the movement deltas
        delta_c1 = c1_ik - c1_base
        delta_thigh = thigh_ik - thigh_base
        delta_tibia = tibia_ik - tibia_base
        
        # 4. Apply the deltas to the User's preferred standing posture
        c1 = 0.0 + delta_c1
        thigh = 0.8 + delta_thigh
        tibia = -0.8 + delta_tibia
        
        return c1, thigh, tibia

    def clamp_and_publish(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()

        for name in ALL_JOINTS:
            val = self.current_angles[name]
            if 'c1' in name:
                val = max(-0.8, min(0.8, val))
            elif 'thigh' in name:
                if name in ['j_thigh_rr', 'j_thigh_lr']:
                    val = max(-0.3, min(1.2, val))
                else:
                    val = max(-0.3, min(1.5, val))
            elif 'tibia' in name:
                val = max(-1.5, min(0.3, val))

            js.name.append(name)
            js.position.append(val)

        self.pub.publish(js)

    def get_smooth_ramp(self, p):
        if p <= 0.0: return 0.0
        if p >= 1.0: return 1.0
        return p * p * (3.0 - 2.0 * p)

    def loop(self):
        rate = rospy.Rate(50) 
        dt = 1.0 / 50.0
        
        # Calculate exactly what the standing target angles should be
        # by running the IK once for a completely flat step (x=0, y=STAND_Y, z=STAND_Z)
        target_stand_angles = {}
        for leg in JOINTS:
            c1, th, ti = self.calculate_leg_ik(leg, 0.0) # phase 0, no velocity
            target_stand_angles[JOINTS[leg][0]] = c1
            target_stand_angles[JOINTS[leg][1]] = th
            target_stand_angles[JOINTS[leg][2]] = ti
        
        while rospy.Time.now().to_sec() == 0:
            rate.sleep()

        start_time = rospy.Time.now().to_sec()
        rospy.loginfo("[FSM] STATE: IDLE / SITTING - Awaiting deploy command...")

        while not rospy.is_shutdown():
            if self.is_sitting:
                break
                
            elapsed = rospy.Time.now().to_sec() - start_time

            if elapsed < 5.0:
                # Phase 1: Pure Zero
                for name in ALL_JOINTS:
                    self.current_angles[name] = 0.0
                self.clamp_and_publish()
                
            elif elapsed < 7.0:
                if elapsed < 5.05:
                    rospy.loginfo("[FSM] STATE: LIFTING / DEPLOYING - Interpolating to Stance...")
                # Phase 2: Smooth Ramp
                progress = (elapsed - 5.0) / 2.0
                scalar = self.get_smooth_ramp(progress)
                
                for leg in JOINTS:
                    self.current_angles[JOINTS[leg][0]] = target_stand_angles[JOINTS[leg][0]] * scalar
                    self.current_angles[JOINTS[leg][1]] = target_stand_angles[JOINTS[leg][1]] * scalar
                    self.current_angles[JOINTS[leg][2]] = target_stand_angles[JOINTS[leg][2]] * scalar
                    
                self.clamp_and_publish()
                
            else:
                if elapsed < 7.05 and self.t == 0.0:
                    rospy.loginfo("[FSM] STATE: STABLE STANDING - Ready for walking commands.")
                # Phase 3: Walking (Aug 25 True Cartesian IK)
                self.vx += (self.target_vx - self.vx) * 0.1
                self.vy += (self.target_vy - self.vy) * 0.1
                self.vt += (self.target_vt - self.vt) * 0.1
                
                if abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.vt) > 0.01:
                    self.t += dt
                else:
                    self.t = 0.0
                    
                cycle = (self.t % CYCLE_TIME) / CYCLE_TIME
                
                for leg in JOINTS:
                    c1, th, ti = self.calculate_leg_ik(leg, cycle)
                    # Apply a very light filter to catch any micro-jerks from the IK
                    self.current_angles[JOINTS[leg][0]] += (c1 - self.current_angles[JOINTS[leg][0]]) * 0.5
                    self.current_angles[JOINTS[leg][1]] += (th - self.current_angles[JOINTS[leg][1]]) * 0.5
                    self.current_angles[JOINTS[leg][2]] += (ti - self.current_angles[JOINTS[leg][2]]) * 0.5
                    
                self.clamp_and_publish()
                
            rate.sleep()

if __name__ == '__main__':
    try:
        SpiderGaitEngine()
    except rospy.ROSInterruptException:
        pass
