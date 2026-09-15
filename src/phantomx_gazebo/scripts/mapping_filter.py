#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
import tf

class MappingFilter:
    def __init__(self):
        rospy.init_node('mapping_filter')
        
        self.lidar_height = 0.15 
        
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.imu_calibrated = False
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        
        rospy.Subscriber('/multiScan/imu', Imu, self.imu_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        
        rospy.Subscriber('/multiScan/scan_fullframe', LaserScan, self.scan_cb)
        rospy.Subscriber('/scan_fullframe', LaserScan, self.scan_cb)
        
        # Subscribe to PointCloud to fix the hardcoded "world" frame bug
        rospy.Subscriber('/cloud_unstructured_fullframe', PointCloud2, self.cloud_cb)
        
        self.scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        self.cloud_pub = rospy.Publisher('/cloud_filtered', PointCloud2, queue_size=10)
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("Mapping Filter active: Direct IMU Floor Rejection. Continuous mapping enabled.")
        
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
        
        if not self.imu_calibrated:
            self.pitch_offset = raw_pitch
            self.roll_offset = raw_roll
            self.imu_calibrated = True
            
        # Subtract offset to ensure we start mathematically flat
        self.current_roll = raw_roll - self.roll_offset
        self.current_pitch = raw_pitch - self.pitch_offset

    def cloud_cb(self, msg):
        time_since_start = (rospy.Time.now() - self.start_time).to_sec()
        if time_since_start < 5.0:
            return
            
        # FORCE the frame to "cloud" because the SICK driver ignores launch arguments and hardcodes "world"!
        msg.header.frame_id = "cloud"
        # Aggressively sync with Jetson clock to prevent TF extrapolation drift
        msg.header.stamp = rospy.Time.now()
        self.cloud_pub.publish(msg)

    def scan_cb(self, msg):
        time_since_start = (rospy.Time.now() - self.start_time).to_sec()
        if time_since_start < 5.0:
            return
            
        new_scan = LaserScan()
        # Aggressively sync with Jetson clock to prevent TF extrapolation drift
        new_scan.header.stamp = rospy.Time.now()
        # FORCE the frame to "cloud_flat" to isolate the mathematically flattened scan from the 3D tilted chassis!
        new_scan.header.frame_id = "cloud_flat"
        
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.angle_increment = msg.angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        
        new_ranges = list(msg.ranges)
        
        p = self.current_pitch
        r = self.current_roll
        
        for i in range(len(new_ranges)):
            dist = new_ranges[i]
            if math.isinf(dist) or math.isnan(dist) or dist < msg.range_min or dist > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            
            # Calculate true height of the laser point
            z_true = self.lidar_height + (-x * math.sin(p)) + (y * math.sin(r))
            
            # If the laser beam hits the floor (Z < 5cm), delete it!
            if z_true < 0.05:
                new_ranges[i] = float('inf')
            else:
                # Project the wall hit to flat 2D distance for slam_toolbox
                x_flat = x * math.cos(p)
                y_flat = y * math.cos(r)
                dist_flat = math.sqrt(x_flat**2 + y_flat**2)
                
                # CRITICAL FIX: The LiDAR sees the robot's own legs! 
                # This causes move_base to think it's trapped in a cage of obstacles.
                # We must filter out any laser hits closer than 0.45m.
                if dist_flat < 0.45:
                    new_ranges[i] = float('inf')
                else:
                    new_ranges[i] = dist_flat
                
        new_scan.ranges = new_ranges
        new_scan.intensities = msg.intensities
        
        self.scan_pub.publish(new_scan)

if __name__ == '__main__':
    try:
        MappingFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
