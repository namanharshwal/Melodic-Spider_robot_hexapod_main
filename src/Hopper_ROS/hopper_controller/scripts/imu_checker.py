#!/usr/bin/env python

from __future__ import print_function

import rospy
import numpy

from math import degrees
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from tf import transformations


def quat_msg_to_array(msg):
    return numpy.array([msg.x, msg.y, msg.z, msg.w])


class ImuChecker(object):
    def __init__(self):
        super(ImuChecker, self).__init__()
        rospy.init_node("IMU_checker")
        self.initial_orientation = Quaternion(0, 0, 0, 1)
        rospy.Subscriber("hopper/imu/data", Imu, self.on_imu_msg, queue_size=10)
        print("Starting")
        rospy.spin()

    def on_imu_msg(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = msg.orientation
            return
        current_orientation = transformations.quaternion_multiply(
            quat_msg_to_array(msg.orientation),
            transformations.quaternion_inverse(quat_msg_to_array(self.initial_orientation)))
        print(map(lambda v: degrees(v), transformations.euler_from_quaternion(current_orientation)))
        print(map(lambda v: degrees(v), transformations.euler_from_quaternion(quat_msg_to_array(msg.orientation))))
        print("")

if __name__ == "__main__":
    ImuChecker()
