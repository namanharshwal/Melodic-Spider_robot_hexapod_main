import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/walker_bridge.py"
with open(file_path, "r") as f:
    content = f.read()

# 1. Update WFunc defaults to smaller swing scale
content = re.sub(r"self\.parameters\['swing_scale'\] = 1", "self.parameters['swing_scale'] = 0.4", content)

# 2. Add offsets in publish_angles
old_publish = """    def publish_angles(self, angles):
        \"\"\"Convert angles dict to JointState and publish\"\"\"
        js = JointState()
        js.header.stamp = rospy.Time.now()
        for name, angle in angles.items():
            js.name.append(name)
            js.position.append(angle)
        self.pub.publish(js)"""

new_publish = """    def publish_angles(self, angles):
        \"\"\"Convert angles dict to JointState and publish with STANDING OFFSETS\"\"\"
        js = JointState()
        js.header.stamp = rospy.Time.now()
        
        STAND_THIGH = 0.6
        STAND_TIBIA = -0.8
        
        for name, angle in angles.items():
            if 'thigh' in name:
                angle += STAND_THIGH
            elif 'tibia' in name:
                angle += STAND_TIBIA
                
            js.name.append(name)
            js.position.append(angle)
            
        self.pub.publish(js)"""

content = content.replace(old_publish, new_publish)

with open(file_path, "w") as f:
    f.write(content)
