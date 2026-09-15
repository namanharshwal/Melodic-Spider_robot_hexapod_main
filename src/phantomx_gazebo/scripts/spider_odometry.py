#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import Imu

class SpiderOdometry:
    def __init__(self):
        rospy.init_node('spider_odometry')
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vth = 0.0
        
        self.use_imu = False
        self.imu_yaw_offset = 0.0
        self.imu_pitch_offset = 0.0
        self.imu_roll_offset = 0.0
        
        self.last_time = rospy.Time.now()
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        # Prioritize the flawless industrial SICK LiDAR IMU, fallback to raw /imu
        rospy.Subscriber('/multiScan/imu', Imu, self.imu_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        # We explicitly DO NOT subscribe to /imu/data because it is magnetically corrupted!
        
    def cmd_cb(self, msg):
        # Perfectly synchronized with spider_gait_engine.py physical stride lengths!
        # CYCLE_TIME reduced from 0.8 to 0.6, so multipliers are increased by 1.33x
        self.target_vx = msg.linear.x * 0.166
        self.target_vy = msg.linear.y * 0.066
        self.target_vth = msg.angular.z * 0.533

    def imu_cb(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        raw_roll = euler[0]
        raw_pitch = euler[1]
        raw_yaw = euler[2]
        
        if not self.use_imu:
            self.imu_yaw_offset = raw_yaw
            self.imu_pitch_offset = raw_pitch
            self.imu_roll_offset = raw_roll
            self.use_imu = True
            
        # The robot MUST depend on the IMU correctly!
        # The IMU tracks physical foot-slipping during strafing which pure math cannot feel.
        self.th = raw_yaw - self.imu_yaw_offset
        self.roll = raw_roll - self.imu_roll_offset
        self.pitch = raw_pitch - self.imu_pitch_offset

    def update(self):
        current_time = rospy.Time.now()
        if self.last_time.to_sec() == 0:
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).to_sec()
        
        # Flawlessly mimic the physical acceleration/inertia of the hexapod legs!
        # The gait engine uses a 0.1 filter at 50Hz (5.0 * dt).
        self.vx += (self.target_vx - self.vx) * 5.0 * dt
        self.vy += (self.target_vy - self.vy) * 5.0 * dt
        self.vth += (self.target_vth - self.vth) * 5.0 * dt
        
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        
        self.x += delta_x
        self.y += delta_y
        
        # 1. SLAM Toolbox requires a strictly 2D odometry frame (Pitch=0, Roll=0).
        odom_quat_2d = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat_2d,
            current_time,
            "base_link",
            "odom"
        )
        
        # 2. Octomap Server needs the TRUE 3D tilt of the robot to map the ceiling and floor!
        odom_quat_3d = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat_3d,
            current_time,
            "base_link_3d",
            "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat_2d))
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

if __name__ == '__main__':
    try:
        node = SpiderOdometry()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            node.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
