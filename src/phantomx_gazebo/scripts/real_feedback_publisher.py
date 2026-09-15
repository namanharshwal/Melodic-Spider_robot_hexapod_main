#!/usr/bin/env python
import rospy
import smbus2
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import servo_feedback_reader as sfr

class FeedbackPublisher:
    def __init__(self):
        rospy.init_node('real_feedback_publisher', anonymous=False)
        self.feedback_topic = rospy.get_param('~feedback_topic', '/phantomx/real_joint_states')
        self.sim_topic = rospy.get_param('~sim_joint_states_topic', '/phantomx/joint_states')
        self.rate_hz = rospy.get_param('~rate', 20)

        # Dictionary to store the latest simulated joint states as fallbacks
        self.latest_sim_joints = {}

        self.pub = rospy.Publisher(self.feedback_topic, JointState, queue_size=10)
        self.sub_sim = rospy.Subscriber(self.sim_topic, JointState, self.sim_callback)

        self.bus = smbus2.SMBus(sfr.I2C_BUS)
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Subscribed to simulated fallback on %s", self.sim_topic)
        rospy.loginfo("Publishing real feedback on %s", self.feedback_topic)
        self.gazebo_pubs = {}

    def sim_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_sim_joints[name] = pos

    def start(self):
        while not rospy.is_shutdown():
            try:
                # Read physical sensor readings
                physical_joints = sfr.read_all_joints(self.bus)

                js = JointState()
                js.header.stamp = rospy.Time.now()

                # Start with default 0.0 for all joints
                merged_joints = {
                    'j_c1_lf': 0.0, 'j_c1_lm': 0.0, 'j_c1_lr': 0.0, 'j_c1_rf': 0.0, 'j_c1_rm': 0.0, 'j_c1_rr': 0.0,
                    'j_thigh_lf': 0.0, 'j_thigh_lm': 0.0, 'j_thigh_lr': 0.0, 'j_thigh_rf': 0.0, 'j_thigh_rm': 0.0, 'j_thigh_rr': 0.0,
                    'j_tibia_lf': 0.0, 'j_tibia_lm': 0.0, 'j_tibia_lr': 0.0, 'j_tibia_rf': 0.0, 'j_tibia_rm': 0.0, 'j_tibia_rr': 0.0
                }
                
                # Apply the latest GUI slider commands as the baseline
                merged_joints.update(self.latest_sim_joints)
                
                # OVERRIDE the GUI sliders with the true physical encoder feedback
                merged_joints.update(physical_joints)

                js.name = list(merged_joints.keys())
                js.position = list(merged_joints.values())
                js.velocity = [0.0] * len(merged_joints)
                js.effort = [0.0] * len(merged_joints)

                self.pub.publish(js)

                # Force the Gazebo simulation to mirror the physical joints
                for name, pos in physical_joints.items():
                    if name not in self.gazebo_pubs:
                        self.gazebo_pubs[name] = rospy.Publisher('/phantomx/' + name + '_position_controller/command', Float64, queue_size=1)
                    self.gazebo_pubs[name].publish(Float64(pos))


            except Exception as e:
                rospy.logwarn("Failed to read sensors: %s", e)

            self.rate.sleep()

if __name__ == '__main__':
    node = FeedbackPublisher()
    node.start()
