#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import time
import sys

class DiagnosticNode:
    def __init__(self):
        rospy.init_node('motor_diagnostic_cli')
        self.cmd = {}
        self.real = {}
        rospy.Subscriber('/phantomx/gui_commands', JointState, self.cmd_cb)
        rospy.Subscriber('/phantomx/real_joint_states', JointState, self.real_cb)

    def cmd_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.cmd[n] = p

    def real_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.real[n] = p

    def run(self):
        print("Gathering data for 3 seconds...")
        time.sleep(3.0)
        
        if not self.cmd or not self.real:
            print("ERROR: Did not receive joint data. Is spider_gait_engine running?")
            sys.exit(1)
            
        print("\n--- MOTOR DIAGNOSTIC REPORT ---")
        failed = []
        working = []
        
        for joint in self.cmd.keys():
            if joint not in self.real:
                continue
                
            cmd_pos = self.cmd[joint]
            real_pos = self.real[joint]
            diff = abs(cmd_pos - real_pos)
            
            if diff > 0.2:
                failed.append("{}: ERR (Cmd: {:.2f}, Real: {:.2f}, Diff: {:.2f} rad)".format(joint, cmd_pos, real_pos, diff))
            else:
                working.append("{}: OK".format(joint))
                
        print("\nWORKING MOTORS (Successfully tracking Safety Engine):")
        for m in sorted(working): print("  - " + m)
        
        print("\nFAILING/BURNT MOTORS (Refusing to track commands):")
        if not failed:
            print("  - NONE! All motors perfectly following the Safety Engine.")
        else:
            for m in sorted(failed): print("  - " + m)

if __name__ == '__main__':
    d = DiagnosticNode()
    d.run()
