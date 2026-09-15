import math

COXA_LEN = 0.054
FEMUR_LEN = 0.082
TIBIA_LEN = 0.133

# Let's try to get femur=0.785 (45deg), tibia perpendicular to ground
z_drop = FEMUR_LEN * math.sin(0.785) + TIBIA_LEN
x_reach = COXA_LEN + FEMUR_LEN * math.cos(0.785)

print("Target Z: {:.3f}, Target X: {:.3f}".format(z_drop, x_reach))

def calculate_ik(x, y, z):
    coxa_angle = math.atan2(y, x)
    L = math.sqrt(x**2 + y**2) - COXA_LEN
    D = math.sqrt(L**2 + z**2)
    
    alpha1 = math.acos((FEMUR_LEN**2 + D**2 - TIBIA_LEN**2) / (2 * FEMUR_LEN * D))
    alpha2 = math.atan2(z, L)
    femur_angle = alpha1 + alpha2
    
    beta = math.acos((FEMUR_LEN**2 + TIBIA_LEN**2 - D**2) / (2 * FEMUR_LEN * TIBIA_LEN))
    
    ros_femur = femur_angle
    ros_tibia = -(math.pi - beta)
    
    return coxa_angle, ros_femur, ros_tibia

c, f, t = calculate_ik(x_reach, 0, z_drop)
print("IK -> Femur: {:.3f}, Tibia: {:.3f}".format(f, t))
