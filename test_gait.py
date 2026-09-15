import math

STAND_THIGH = 0.6
STAND_TIBIA = -0.8
STAND_C1    = 0.0

ALL_JOINTS = [
    'j_c1_lf', 'j_thigh_lf', 'j_tibia_lf',
    'j_c1_lm', 'j_thigh_lm', 'j_tibia_lm',
]

def get_trapezoidal_progress(p):
    if p <= 0.0: return 0.0
    if p >= 1.0: return 1.0
    V_max = 1.0 / 0.85
    if p <= 0.15: return 0.5 * V_max * (p ** 2) / 0.15
    elif p <= 0.85: return (0.5 * V_max * 0.15) + V_max * (p - 0.15)
    else: return 1.0 - (0.5 * V_max * ((1.0 - p) ** 2) / 0.15)

current_angles = {j: 0.0 for j in ALL_JOINTS}

print("Phase 1: 0-5s")
print(current_angles['j_thigh_lf'])

print("Phase 2: 5-7s")
for i in range(11):
    elapsed = 5.0 + i * 0.2
    progress = (elapsed - 5.0) / 2.0
    scalar = get_trapezoidal_progress(progress)
    current_angles['j_thigh_lf'] = STAND_THIGH * scalar
    print("elapsed {:.1f}, scalar {:.2f}, thigh {:.2f}".format(elapsed, scalar, current_angles['j_thigh_lf']))
