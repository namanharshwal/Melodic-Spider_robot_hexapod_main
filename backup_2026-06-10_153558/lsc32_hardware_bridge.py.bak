#!/usr/bin/env python3
"""
PhantomX Hexapod - LSC32 Hardware Bridge
Bridges ROS simulation joint states -> real servos via LSC-32 v1.4
USB-TTL adapter: /dev/ttyUSB0 @ 9600 baud
"""

import rospy
import serial
import math
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# ?????????????????????????????????????????????????????
# UART CONFIG
# ?????????????????????????????????????????????????????
UART_PORT = '/dev/ttyUSB0'
UART_BAUD = 9600

# ?????????????????????????????????????????????????????
# SERVO CHANNEL MAP (your confirmed wiring)
# LSC-32 channel ? RDS3115MG servo ID
# ?????????????????????????????????????????????????????
# PhantomX ROS joint name ? LSC-32 servo ID
JOINT_TO_SERVO = {
    # LEFT LEGS (S8?S0, front to rear)
    'j_c1_lf':    8,   # Front-Left  Coxa
    'j_thigh_lf': 7,   # Front-Left  Femur
    'j_tibia_lf': 6,   # Front-Left  Tibia

    'j_c1_lm':    5,   # Middle-Left Coxa
    'j_thigh_lm': 4,   # Middle-Left Femur
    'j_tibia_lm': 3,   # Middle-Left Tibia

    'j_c1_lr':    2,   # Rear-Left   Coxa
    'j_thigh_lr': 1,   # Rear-Left   Femur
    'j_tibia_lr': 0,   # Rear-Left   Tibia

    # RIGHT LEGS (S22?S30, front to rear)
    'j_c1_rf':    22,  # Front-Right  Coxa
    'j_thigh_rf': 23,  # Front-Right  Femur
    'j_tibia_rf': 24,  # Front-Right  Tibia

    'j_c1_rm':    25,  # Middle-Right Coxa
    'j_thigh_rm': 26,  # Middle-Right Femur
    'j_tibia_rm': 27,  # Middle-Right Tibia

    'j_c1_rr':    28,  # Rear-Right   Coxa
    'j_thigh_rr': 29,  # Rear-Right   Femur
    'j_tibia_rr': 30,  # Rear-Right   Tibia
}

# ?????????????????????????????????????????????????????
# JOINT LIMITS (from PhantomX URDF ? radians)
# ?????????????????????????????????????????????????????
JOINT_LIMITS = {
    'coxa':  (-0.785, 0.785),   # ?45 degrees
    'femur': (-0.523, 0.523),   # ?30 degrees
    'tibia': (-0.785, 0.785),   # ?45 degrees
}

def get_joint_type(joint_name):
    if 'c1' in joint_name:    return 'coxa'
    if 'thigh' in joint_name: return 'femur'
    if 'tibia' in joint_name: return 'tibia'
    return 'coxa'

def rad_to_lsc(angle_rad, joint_type):
    """
    Convert joint angle (radians) to LSC-32 position (0-1000)
    Center (0 rad) = 500 on LSC-32
    RDS3115MG full range = 0-180 degrees = 0-1000 LSC units
    """
    lo, hi = JOINT_LIMITS[joint_type]
    # Clamp to joint limits
    angle_rad = max(lo, min(hi, angle_rad))
    # Map from [lo, hi] to [0, 1000]
    lsc_pos = int(((angle_rad - lo) / (hi - lo)) * 1000)
    return max(0, min(1000, lsc_pos))


class LSC32HardwareBridge:

    def __init__(self):
        rospy.init_node('lsc32_hardware_bridge', anonymous=False)
        rospy.loginfo("LSC-32 Hardware Bridge starting...")

        # Open serial port
        try:
            self.ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
            rospy.loginfo("Serial connected: " + UART_PORT + " @ " + str(UART_BAUD))
        except serial.SerialException as e:
            rospy.logerr("SERIAL FAILED: " + str(e))
            raise SystemExit(1)

        # Give LSC-32 time to initialize
        time.sleep(1.0)

        # Stand up ? center all servos on startup
        rospy.loginfo("Centering all 18 servos (standing position)...")
        self.center_all(move_time=2000)
        rospy.loginfo("Robot ready!")

        # Subscribe to PhantomX joint states from Gazebo/controller
        self.sub = rospy.Subscriber(
            '/phantomx/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Subscribed to /phantomx/joint_states")
        rospy.loginfo("Simulation -> Hardware bridge ACTIVE")

    def send_servo(self, servo_id, position, move_time=200):
        """Send single servo command to LSC-32"""
        position = max(0, min(1000, position))
        packet = bytes([
            0x55, 0x55,              # Header
            0x08,                    # Length
            0x03,                    # CMD: SERVO_MOVE_TIME_WRITE
            0x01,                    # Count: 1 servo
            move_time & 0xFF,        # Time LOW
            (move_time >> 8) & 0xFF, # Time HIGH
            servo_id,                # Servo ID
            position & 0xFF,         # Position LOW
            (position >> 8) & 0xFF   # Position HIGH
        ])
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            rospy.logerr("Write error: " + str(e))

    def center_all(self, move_time=1500):
        """Move all 18 servos to center position 500"""
        for servo_id in JOINT_TO_SERVO.values():
            self.send_servo(servo_id, 500, move_time)
            time.sleep(0.05)
        time.sleep(float(move_time) / 1000.0 + 0.5)

    def joint_state_callback(self, msg):
        """
        Receive joint states from simulation,
        convert angles, send to real servos
        """
        for i, joint_name in enumerate(msg.name):
            if joint_name in JOINT_TO_SERVO:
                servo_id   = JOINT_TO_SERVO[joint_name]
                angle_rad  = msg.position[i]
                joint_type = get_joint_type(joint_name)
                lsc_pos    = rad_to_lsc(angle_rad, joint_type)
                # 150ms move time for smooth tracking
                self.send_servo(servo_id, lsc_pos, move_time=150)

    def shutdown(self):
        rospy.loginfo("Shutting down ? centering all servos...")
        self.center_all(move_time=1000)
        self.ser.close()
        rospy.loginfo("LSC-32 bridge closed.")


if __name__ == '__main__':
    bridge = LSC32HardwareBridge()
    rospy.on_shutdown(bridge.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
