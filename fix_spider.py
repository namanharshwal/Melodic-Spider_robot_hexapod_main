import re

file_path = "/home/nammy/SD_Storage/catkin_ws/src/phantomx_gazebo/scripts/spider_gait_engine.py"
with open(file_path, "r") as f:
    content = f.read()

correct_generate = """    def generate(self):
        self.pfn = {}
        self.afn = {}

        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = -self.parameters['swing_scale']

        f2 = f1.clone()
        f2.scale = 0

        f3 = f1.clone()
        f3.scale *= -1

        f4 = f2.clone()
        f4.scale *= -1

        zero = WJFunc()
        zero.scale = 0

        self.set_func('j_thigh', f1, f2)
        self.set_func('j_tibia', f3, f4)
        self.set_func('j_c1', zero, zero)"""

# Use regex to replace the generate function
content = re.sub(r"    def generate\(self\):.*?self\.set_func\('j_c1', zero, zero\)", correct_generate, content, flags=re.DOTALL)

with open(file_path, "w") as f:
    f.write(content)
