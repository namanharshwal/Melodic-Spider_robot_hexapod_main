#!/usr/bin/env python
"""
Walker Bridge: Complete lifecycle manager for the spider robot.
Phase 1 (0-5s):   All servos at ZERO (flat/rest)
Phase 2 (5-7s):   Trapezoidal ramp from zero to standing pose
Phase 3 (7s+):    Proven sinusoidal tripod gait from walker.py

Standing Pose: STAND_THIGH=0.6, STAND_TIBIA=-0.8 (confirmed perfect)
"""

import rospy
import math
import time
from threading import Thread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


# =========================================================================
# CONFIRMED PERFECT STANDING POSE
# =========================================================================
STAND_THIGH = 0.6
STAND_TIBIA = -0.8
STAND_C1    = 0.0

# All 18 joint names
ALL_JOINTS = [
    'j_c1_lf', 'j_thigh_lf', 'j_tibia_lf',
    'j_c1_lm', 'j_thigh_lm', 'j_tibia_lm',
    'j_c1_lr', 'j_thigh_lr', 'j_tibia_lr',
    'j_c1_rf', 'j_thigh_rf', 'j_tibia_rf',
    'j_c1_rm', 'j_thigh_rm', 'j_tibia_rm',
    'j_c1_rr', 'j_thigh_rr', 'j_tibia_rr',
]


# =========================================================================
# PROVEN WALKING MATH (Extracted directly from walker.py)
# =========================================================================

class WJFunc:
    """Walk Joint Function - Pure sinusoidal joint generator"""
    def __init__(self):
        self.offset = 0
        self.scale = 1
        self.in_offset = 0
        self.in_scale = 1

    def get(self, x):
        f = math.sin(self.in_offset + self.in_scale * x)
        return self.offset + self.scale * f

    def clone(self):
        z = WJFunc()
        z.offset = self.offset
        z.scale = self.scale
        z.in_offset = self.in_offset
        z.in_scale = self.in_scale
        return z


class WFunc:
    """Walk Function - Proven PhantomX tripod gait generator"""
    def __init__(self):
        self.parameters = {}
        self.parameters['swing_scale'] = 0.4
        self.parameters['vx_scale'] = 0.5
        self.parameters['vy_scale'] = 0.5
        self.parameters['vt_scale'] = 0.4
        self.generate()

    def generate(self):
        self.pfn = {}
        self.afn = {}

        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = -self.parameters['swing_scale']

        f2 = f1.clone()
        f2.scale = 0

        f3 = f1.clone()
        f3.scale *= -1

        f4 = f2.clone()
        f3.scale *= -1

        zero = WJFunc()
        zero.scale = 0

        self.set_func('j_thigh', f1, f2)
        self.set_func('j_tibia', f3, f4)
        self.set_func('j_c1', zero, zero)

    def set_func(self, joint, fp, fa):
        for leg in ['lf', 'rm', 'lr']:
            j = joint + '_' + leg
            self.pfn[j] = fp
            self.afn[j] = fa
        for leg in ['rf', 'lm', 'rr']:
            j = joint + '_' + leg
            self.pfn[j] = fa
            self.afn[j] = fp

    def get(self, phase, x, velocity):
        angles = {}
        for j in self.pfn.keys():
            if phase:
                angles[j] = self.pfn[j].get(x)
            else:
                angles[j] = self.afn[j].get(x)
        self.apply_velocity(angles, velocity, phase, x)
        return angles

    def apply_velocity(self, angles, velocity, phase, x):
        # VX (Forward/Backward)
        v = velocity[0] * self.parameters['vx_scale']
        d = (x * 2 - 1) * v
        if phase:
            angles['j_c1_lf'] -= d
            angles['j_c1_rm'] += d
            angles['j_c1_lr'] -= d
            angles['j_c1_rf'] -= d
            angles['j_c1_lm'] += d
            angles['j_c1_rr'] -= d
        else:
            angles['j_c1_lf'] += d
            angles['j_c1_rm'] -= d
            angles['j_c1_lr'] += d
            angles['j_c1_rf'] += d
            angles['j_c1_lm'] -= d
            angles['j_c1_rr'] += d

        # VT (Turning)
        v = velocity[2] * self.parameters['vt_scale']
        d = (x * 2 - 1) * v
        if phase:
            angles['j_c1_lf'] += d
            angles['j_c1_rm'] += d
            angles['j_c1_lr'] += d
            angles['j_c1_rf'] -= d
            angles['j_c1_lm'] -= d
            angles['j_c1_rr'] -= d
        else:
            angles['j_c1_lf'] -= d
            angles['j_c1_rm'] -= d
            angles['j_c1_lr'] -= d
            angles['j_c1_rf'] += d
            angles['j_c1_lm'] += d
            angles['j_c1_rr'] += d


# =========================================================================
# TRAPEZOIDAL RAMP (15% ramp up, 70% linear, 15% ramp down)
# =========================================================================

def trapezoidal(p):
    if p <= 0.0: return 0.0
    if p >= 1.0: return 1.0
    V_max = 1.0 / 0.85
    if p <= 0.15:
        return 0.5 * V_max * (p ** 2) / 0.15
    elif p <= 0.85:
        return (0.5 * V_max * 0.15) + V_max * (p - 0.15)
    else:
        p_rem = 1.0 - p
        return 1.0 - (0.5 * V_max * (p_rem ** 2) / 0.15)


# =========================================================================
# WALKER BRIDGE NODE
# =========================================================================

class WalkerBridge:
    def __init__(self):
        rospy.init_node('walker_bridge')

        self.pub = rospy.Publisher('/phantomx/gui_commands', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)

        self.velocity = [0.0, 0.0, 0.0]
        self.target_velocity = [0.0, 0.0, 0.0]
        self.walking = False
        self.func = WFunc()

        self._th_walk = Thread(target=self._do_walk)
        self._th_walk.daemon = True
        self._th_walk.start()

        rospy.loginfo("Walker Bridge: PROVEN sinusoidal tripod gait active!")

    def cmd_cb(self, msg):
        self.target_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not self.walking:
            self.walking = True

    def get_standing_angle(self, joint_name):
        """Return the confirmed perfect standing angle for a joint"""
        if 'thigh' in joint_name:
            return STAND_THIGH
        elif 'tibia' in joint_name:
            return STAND_TIBIA
        else:
            return STAND_C1

    def publish_zero(self):
        """Publish all joints at zero (flat/rest position)"""
        js = JointState()
        js.header.stamp = rospy.Time.now()
        for name in ALL_JOINTS:
            js.name.append(name)
            js.position.append(0.0)
        self.pub.publish(js)

    def publish_ramp(self, scalar):
        """Publish standing pose scaled by scalar (0.0 to 1.0)"""
        js = JointState()
        js.header.stamp = rospy.Time.now()
        for name in ALL_JOINTS:
            target = self.get_standing_angle(name)
            js.name.append(name)
            js.position.append(target * scalar)
        self.pub.publish(js)

    def publish_walk_angles(self, angles):
        """Publish walker angles with standing offsets applied"""
        js = JointState()
        js.header.stamp = rospy.Time.now()
        for name, angle in angles.items():
            if 'thigh' in name:
                angle += STAND_THIGH
            elif 'tibia' in name:
                angle += STAND_TIBIA
            js.name.append(name)
            js.position.append(angle)
        self.pub.publish(js)

    def update_velocity(self, target, n):
        a = 3.0 / float(n)
        b = 1.0 - a
        self.velocity = [a * t + b * v for (t, v) in zip(target, self.velocity)]

    def is_walking(self):
        e = 0.02
        for v in self.velocity:
            if abs(v) > e:
                return True
        return False

    def _do_walk(self):
        r = rospy.Rate(50)
        n = 50
        p = True
        i = 0

        # Wait for ROS time
        while rospy.Time.now().to_sec() == 0:
            r.sleep()

        start_time = rospy.Time.now().to_sec()
        rospy.loginfo('Walker Bridge: Phase 1 - Holding ZERO for 5 seconds...')

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start_time

            # =============================================
            # PHASE 1: Hold all servos at ZERO (0-5 seconds)
            # =============================================
            if elapsed < 5.0:
                self.publish_zero()
                r.sleep()
                continue

            # =============================================
            # PHASE 2: Trapezoidal ramp to standing (5-7 seconds)
            # =============================================
            if elapsed < 7.0:
                progress = (elapsed - 5.0) / 2.0
                scalar = trapezoidal(progress)
                self.publish_ramp(scalar)
                if elapsed < 5.1:
                    rospy.loginfo('Walker Bridge: Phase 2 - Ramping to STANDING pose...')
                r.sleep()
                continue

            # =============================================
            # PHASE 3: Walking (7+ seconds)
            # =============================================
            if elapsed < 7.1:
                rospy.loginfo('Walker Bridge: Phase 3 - READY FOR MOVEMENT!')

            # Smooth velocity filter
            if not self.walking:
                self.target_velocity = [0, 0, 0]
            self.update_velocity(self.target_velocity, n)

            if not self.is_walking() and i == 0:
                # Standing still - hold the perfect standing pose
                ready = self.func.get(True, 0, [0, 0, 0])
                self.publish_walk_angles(ready)
                r.sleep()
                continue

            x = float(i) / n
            angles = self.func.get(p, x, self.velocity)
            self.publish_walk_angles(angles)

            i += 1
            if i > n:
                i = 0
                p = not p

            r.sleep()


if __name__ == '__main__':
    try:
        bridge = WalkerBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
