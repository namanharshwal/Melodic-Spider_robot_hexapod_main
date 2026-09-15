#!/usr/bin/env python
"""
LSC-32 Hardware Bridge driven by joint_state_publisher_gui sliders.
Subscribes to /phantomx/joint_states (from Gazebo OR joint_state_publisher_gui)
and drives real servos via UART at 9600 baud.
"""
import rospy
import serial
import time
from sensor_msgs.msg import JointState

UART_PORT = '/dev/ttyUSB0'
UART_BAUD = 9600

LEG_JOINTS = [
    'j_c1_lf', 'j_c1_rf', 'j_c1_lm', 'j_c1_rm', 'j_c1_lr', 'j_c1_rr',
    'j_thigh_lf', 'j_thigh_rf', 'j_thigh_lm', 'j_thigh_rm', 'j_thigh_lr', 'j_thigh_rr',
    'j_tibia_lf', 'j_tibia_rf', 'j_tibia_lm', 'j_tibia_rm', 'j_tibia_lr', 'j_tibia_rr'
]

JOINT_LIMITS = {
    'coxa':  (-0.785, 0.785),
    'femur': (-0.650, 0.650),
    'tibia': (-0.785, 0.785),
}

JOINT_TO_SERVO = {
    'j_c1_lf': 8, 'j_thigh_lf': 7, 'j_tibia_lf': 6,
    'j_c1_lm': 5, 'j_thigh_lm': 4, 'j_tibia_lm': 3,
    'j_c1_lr': 2, 'j_thigh_lr': 1, 'j_tibia_lr': 0,
    'j_c1_rf': 22, 'j_thigh_rf': 23, 'j_tibia_rf': 24,
    'j_c1_rm': 25, 'j_thigh_rm': 26, 'j_tibia_rm': 27,
    'j_c1_rr': 28, 'j_thigh_rr': 29, 'j_tibia_rr': 30,
}

SERVO_DIRECTION = {
    'j_c1_lf': 1,  'j_thigh_lf': -1, 'j_tibia_lf': 1,
    'j_c1_lm': 1,  'j_thigh_lm': -1, 'j_tibia_lm': 1,
    'j_c1_lr': 1,  'j_thigh_lr': -1, 'j_tibia_lr': 1,
    'j_c1_rf': 1,  'j_thigh_rf': 1,  'j_tibia_rf': 1,
    'j_c1_rm': 1,  'j_thigh_rm': 1,  'j_tibia_rm': 1,
    'j_c1_rr': 1,  'j_thigh_rr': 1,  'j_tibia_rr': 1,
}

FEMUR_GAIN = 1.35
SERVO_OFFSET = {k: 0 for k in JOINT_TO_SERVO.keys()}

def get_joint_type(name):
    if 'c1' in name:
        return 'coxa'
    if 'thigh' in name:
        return 'femur'
    return 'tibia'

def rad_to_lsc(angle_rad, joint_name):
    jt = get_joint_type(joint_name)

    if jt == 'femur':
        angle_rad *= FEMUR_GAIN

    lo, hi = JOINT_LIMITS[jt]
    angle_rad = max(lo, min(hi, angle_rad))

    if SERVO_DIRECTION.get(joint_name, 1) == -1:
        angle_rad = -angle_rad

    pos = int(((angle_rad - lo) / (hi - lo)) * 1000)
    pos += SERVO_OFFSET.get(joint_name, 0)
    return max(0, min(1000, pos))

class Bridge:
    def __init__(self):
        rospy.init_node('lsc32_jspgui_bridge', anonymous=False)
        rospy.loginfo('Connecting to LSC-32 @ 9600 baud...')
        self.ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        time.sleep(1.0)
        self.center_all(1500)
        topic = rospy.get_param('~joint_topic', '/phantomx/joint_states')
        rospy.Subscriber(topic, JointState, self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo('Subscribed to %s', topic)
        rospy.loginfo('Bridge active ? move sliders or run simulation')

    def send(self, sid, pos, t=90):
        pos = max(0, min(1000, int(pos)))
        self.ser.write(bytearray([
            0x55, 0x55, 0x08, 0x03, 0x01,
            t & 0xFF, (t >> 8) & 0xFF,
            sid, pos & 0xFF, (pos >> 8) & 0xFF
        ]))

    def center_all(self, t=1500):
        for jname in JOINT_TO_SERVO:
            self.send(JOINT_TO_SERVO[jname], 500, t)
            time.sleep(0.05)
        time.sleep(t / 1000.0 + 0.5)

    def cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in JOINT_TO_SERVO:
                sid = JOINT_TO_SERVO[name]
                pos = rad_to_lsc(msg.position[i], name)
                self.send(sid, pos, 80)

    def shutdown(self):
        try:
            self.center_all(1000)
            self.ser.close()
        except:
            pass

if __name__ == '__main__':
    b = Bridge()
    rospy.on_shutdown(b.shutdown)
    rospy.spin()
