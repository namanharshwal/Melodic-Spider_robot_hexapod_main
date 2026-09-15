#!/usr/bin/env python
"""
Motor Safety Governor: PASSIVE safety filter between gait engine and hardware.
- Absolute joint clamps
- Rate-of-change limiter (prevents stall current spikes)
- 180-degree servo awareness for j_thigh_rr and j_thigh_lr

NO startup ramp here (walker_bridge handles lifecycle).
NO IMU influence (SLAM only).
"""

import rospy
from sensor_msgs.msg import JointState

# =========================================================================
# ABSOLUTE JOINT SAFETY LIMITS (radians)
# =========================================================================
JOINT_LIMITS = {
    'c1':    (-0.6, 0.6),
    'thigh': (-0.5, 1.2),
    'tibia': (-1.3, 0.1),
}

# 180-degree servos have tighter effective range
SERVO_180_JOINTS = ['j_thigh_rr', 'j_thigh_lr']
SERVO_180_LIMITS = (-0.5, 0.9)

# Maximum angular change per tick at 50Hz
# 0.2 rad/tick = 10 rad/s max velocity (generous but safe)
MAX_DELTA_PER_TICK = 0.2


class MotorSafetyGovernor:
    def __init__(self):
        rospy.init_node('motor_safety_governor')

        self.pub = rospy.Publisher('/phantomx/safe_joint_states', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/phantomx/gui_commands', JointState, self.joint_cb, queue_size=1)

        self.prev_angles = {}

        rospy.loginfo("Motor Safety Governor: ACTIVE (passive filter only)")
        rospy.loginfo("  - Absolute joint clamps: ON")
        rospy.loginfo("  - Rate-of-change limiter: ON (max %.2f rad/tick)" % MAX_DELTA_PER_TICK)
        rospy.loginfo("  - 180-deg servo awareness: ON for %s" % str(SERVO_180_JOINTS))
        rospy.loginfo("  - IMU influence on motors: DISABLED (SLAM only)")

    def get_joint_type(self, name):
        if 'c1' in name:
            return 'c1'
        elif 'thigh' in name:
            return 'thigh'
        elif 'tibia' in name:
            return 'tibia'
        return None

    def clamp_angle(self, angle, joint_name):
        jtype = self.get_joint_type(joint_name)
        if jtype is None:
            return angle

        if joint_name in SERVO_180_JOINTS:
            lo, hi = SERVO_180_LIMITS
        else:
            lo, hi = JOINT_LIMITS[jtype]

        return max(lo, min(hi, angle))

    def rate_limit(self, angle, joint_name):
        if joint_name not in self.prev_angles:
            self.prev_angles[joint_name] = 0.0

        prev = self.prev_angles[joint_name]
        delta = angle - prev

        if abs(delta) > MAX_DELTA_PER_TICK:
            if delta > 0:
                angle = prev + MAX_DELTA_PER_TICK
            else:
                angle = prev - MAX_DELTA_PER_TICK

        self.prev_angles[joint_name] = angle
        return angle

    def joint_cb(self, msg):
        safe_js = JointState()
        safe_js.header.stamp = rospy.Time.now()

        for i, joint_name in enumerate(msg.name):
            raw_angle = msg.position[i]

            # LAYER 1: Absolute Joint Clamps
            safe_angle = self.clamp_angle(raw_angle, joint_name)

            # LAYER 2: Rate-of-Change Limiter
            safe_angle = self.rate_limit(safe_angle, joint_name)

            safe_js.name.append(joint_name)
            safe_js.position.append(safe_angle)

        self.pub.publish(safe_js)


if __name__ == '__main__':
    try:
        governor = MotorSafetyGovernor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
