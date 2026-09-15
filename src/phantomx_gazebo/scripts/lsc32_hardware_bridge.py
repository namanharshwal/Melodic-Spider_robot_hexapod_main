#!/usr/bin/env python
"""
PhantomX Hexapod - LSC32 Hardware Bridge
Bridges ROS simulation joint states -> real servos via LSC-32 v1.4
Fixed UART Saturation & Interpolation Jitter Bugs
"""

import rospy
import serial
import time
from sensor_msgs.msg import JointState

UART_PORT = '/dev/ttyUSB0'
UART_BAUD = 9600

JOINT_TO_SERVO = {
    'j_c1_lf': 8,  'j_thigh_lf': 7,  'j_tibia_lf': 6,
    'j_c1_lm': 5,  'j_thigh_lm': 4,  'j_tibia_lm': 3,
    'j_c1_lr': 2,  'j_thigh_lr': 1,  'j_tibia_lr': 0,
    'j_c1_rf': 22, 'j_thigh_rf': 23, 'j_tibia_rf': 24,
    'j_c1_rm': 25, 'j_thigh_rm': 26, 'j_tibia_rm': 27,
    'j_c1_rr': 28, 'j_thigh_rr': 29, 'j_tibia_rr': 30,
}

def rad_to_lsc(angle_rad, joint_name):
    angle_rad = max(-1.3, min(1.3, angle_rad))
    if joint_name in ['j_thigh_rr', 'j_thigh_lr']:
        lsc_pos = 1500 + int((angle_rad / 1.57079) * 1000)
    else:
        lsc_pos = 1500 + int((angle_rad / 2.35619) * 1000)
    return max(500, min(2500, lsc_pos))

class LSC32HardwareBridge:
    def __init__(self):
        rospy.init_node('lsc32_hardware_bridge', anonymous=False)
        rospy.loginfo("LSC-32 Hardware Bridge starting...")

        try:
            self.ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
            rospy.loginfo("Serial connected: " + UART_PORT + " @ " + str(UART_BAUD))
        except serial.SerialException as e:
            rospy.logerr("SERIAL FAILED: " + str(e))
            raise SystemExit(1)

        time.sleep(1.0)

        # EXACT ZERO POSITION
        rospy.loginfo("Centering all servos to ZERO (1500)...")
        self.center_all(move_time=1000)
        time.sleep(1.0)
        rospy.loginfo("Robot ready!")

        self.last_write_time = time.time()
        
        # INCREASED RATE LIMIT TO PREVENT UART CORRUPTION
        # 9600 baud = 960 bytes/sec. A 61 byte packet takes 63.5ms.
        # Sending every 100ms (10Hz) guarantees no buffer overflow!
        self.rate_limit_sec = 0.10

        joint_topic = rospy.get_param('~input_joint_states_topic', '/phantomx/gui_commands')
        self.sub = rospy.Subscriber(joint_topic, JointState, self.joint_state_callback, queue_size=1)

    def send_servos_multi(self, servos_dict, move_time=100):
        count = len(servos_dict)
        if count == 0: return
            
        length = count * 3 + 5
        packet = bytearray([
            0x55, 0x55,
            length,
            0x03,
            count,
            move_time & 0xFF,
            (move_time >> 8) & 0xFF
        ])
        
        for servo_id, position in servos_dict.items():
            position = max(500, min(2500, position))
            packet.extend([
                servo_id,
                position & 0xFF,
                (position >> 8) & 0xFF
            ])
            
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            rospy.logerr("Write error: " + str(e))

    def center_all(self, move_time=1000):
        servos_to_move = {sid: 1500 for sid in JOINT_TO_SERVO.values()}
        self.send_servos_multi(servos_to_move, move_time)

    def joint_state_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_write_time < self.rate_limit_sec:
            return
            
        self.last_write_time = current_time
        
        servos_to_move = {}
        for i, joint_name in enumerate(msg.name):
            if joint_name in JOINT_TO_SERVO:
                servo_id   = JOINT_TO_SERVO[joint_name]
                angle_rad  = msg.position[i]
                servos_to_move[servo_id] = rad_to_lsc(angle_rad, joint_name)
                
        if servos_to_move:
            # move_time is matched EXACTLY to rate_limit to prevent servo stutter/randomness
            self.send_servos_multi(servos_to_move, move_time=int(self.rate_limit_sec * 1000))

    def shutdown(self):
        rospy.loginfo("Shutting down - centering all servos...")
        self.center_all(move_time=1000)
        self.ser.close()

if __name__ == '__main__':
    bridge = LSC32HardwareBridge()
    rospy.on_shutdown(bridge.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
