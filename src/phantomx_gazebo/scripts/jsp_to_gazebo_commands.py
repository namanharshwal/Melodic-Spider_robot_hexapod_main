#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_CONTROLLER_TOPICS = {
    'j_c1_lf':    '/phantomx/j_c1_lf_position_controller/command',
    'j_thigh_lf': '/phantomx/j_thigh_lf_position_controller/command',
    'j_tibia_lf': '/phantomx/j_tibia_lf_position_controller/command',

    'j_c1_lm':    '/phantomx/j_c1_lm_position_controller/command',
    'j_thigh_lm': '/phantomx/j_thigh_lm_position_controller/command',
    'j_tibia_lm': '/phantomx/j_tibia_lm_position_controller/command',

    'j_c1_lr':    '/phantomx/j_c1_lr_position_controller/command',
    'j_thigh_lr': '/phantomx/j_thigh_lr_position_controller/command',
    'j_tibia_lr': '/phantomx/j_tibia_lr_position_controller/command',

    'j_c1_rf':    '/phantomx/j_c1_rf_position_controller/command',
    'j_thigh_rf': '/phantomx/j_thigh_rf_position_controller/command',
    'j_tibia_rf': '/phantomx/j_tibia_rf_position_controller/command',

    'j_c1_rm':    '/phantomx/j_c1_rm_position_controller/command',
    'j_thigh_rm': '/phantomx/j_thigh_rm_position_controller/command',
    'j_tibia_rm': '/phantomx/j_tibia_rm_position_controller/command',

    'j_c1_rr':    '/phantomx/j_c1_rr_position_controller/command',
    'j_thigh_rr': '/phantomx/j_thigh_rr_position_controller/command',
    'j_tibia_rr': '/phantomx/j_tibia_rr_position_controller/command',
}

class JSPToGazebo:
    def __init__(self):
        rospy.init_node('jsp_to_gazebo_commands', anonymous=False)
        input_topic = rospy.get_param('~input_joint_states_topic', '/phantomx/gui_joint_states')

        self.pubs = {
            joint: rospy.Publisher(topic, Float64, queue_size=1)
            for joint, topic in JOINT_CONTROLLER_TOPICS.items()
        }

        rospy.Subscriber(input_topic, JointState, self.cb, queue_size=1)
        rospy.loginfo("jsp_to_gazebo_commands listening on %s", input_topic)

    def cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        for joint, pub in self.pubs.items():
            if joint in name_to_pos:
                pub.publish(Float64(name_to_pos[joint]))

if __name__ == '__main__':
    JSPToGazebo()
    rospy.spin()
