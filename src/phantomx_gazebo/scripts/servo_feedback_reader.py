#!/usr/bin/env python
"""
Reads RDS3115 potentiometer taps via ADS1115.
Supports direct connection (no MUX) for testing or multiplexed TCA9548A mode.
"""

import time
import smbus2

# Configuration Flags
USE_MUX = True  # Set to True when the TCA9548A I2C multiplexer is connected

I2C_BUS = 1
TCA_ADDR = 0x70
ADS_ADDR = 0x48

ADS_REG_CONFIG = 0x01
ADS_REG_CONVERSION = 0x00

GAIN_FSR = 4.096
ADS_RESOLUTION = 32768.0

CONFIG_OS_SINGLE = 0x8000
CONFIG_MUX_AIN0 = 0x4000
CONFIG_MUX_AIN1 = 0x5000
CONFIG_MUX_AIN2 = 0x6000
CONFIG_MUX_AIN3 = 0x7000
CONFIG_PGA_4_096V = 0x0200
CONFIG_MODE_SINGLE = 0x0100
CONFIG_DR_860SPS = 0x00E0
CONFIG_COMP_DISABLE = 0x0003

MUX_MAP = [CONFIG_MUX_AIN0, CONFIG_MUX_AIN1, CONFIG_MUX_AIN2, CONFIG_MUX_AIN3]

# MUX Mode Map (6 ADS1115 modules on channels 0, 1, 2, 5, 6, 7)
# Assuming wiring matches previous standard: AIN0=Coxa, AIN1=Thigh, AIN2=Tibia
JOINT_MAP_MUX = {
    0: {0: 'j_c1_lf', 1: 'j_thigh_lf', 2: 'j_tibia_lf'},
    1: {0: 'j_c1_lm', 1: 'j_thigh_lm', 2: 'j_tibia_lm'},
    2: {0: 'j_c1_lr', 1: 'j_thigh_lr', 2: 'j_tibia_lr'},
    5: {0: 'j_c1_rf', 1: 'j_thigh_rf', 2: 'j_tibia_rf'},
    6: {0: 'j_c1_rm', 1: 'j_thigh_rm', 2: 'j_tibia_rm'},
    7: {0: 'j_c1_rr', 1: 'j_thigh_rr', 2: 'j_tibia_rr'},
}

# Direct Mode Map (No MUX - Single ADS1115 connected directly to bus 1)
JOINT_MAP_DIRECT = {
    0: 'j_c1_lr',     # AIN0 is physically connected to Servo 2 (Left Rear Coxa)
    1: 'j_thigh_lr',  # AIN1 is physically connected to Servo 1 (Left Rear Femur)
    2: 'j_tibia_lr',  # AIN2 is physically connected to Servo 0 (Left Rear Tibia)
}

# Per-joint calibration: measured voltage at mechanical min/max + angle limits (radians)
CALIBRATION = {
    joint: {'v_min': 0.08, 'v_max': 3.01, 'theta_min': -2.35619, 'theta_max': 2.35619}
    for joint in (['j_c1_lf', 'j_thigh_lf', 'j_tibia_lf', 'j_c1_lm', 'j_thigh_lm', 'j_tibia_lm', 'j_c1_lr', 'j_thigh_lr', 'j_tibia_lr',
                   'j_c1_rf', 'j_thigh_rf', 'j_tibia_rf', 'j_c1_rm', 'j_thigh_rm', 'j_tibia_rm',
                   'j_c1_rr', 'j_thigh_rr', 'j_tibia_rr'])
}
# Locked Calibration for the 270-Degree Encoder Servo (Servo 0)
CALIBRATION['j_tibia_lr'] = {'v_min': 0.081, 'v_max': 3.006, 'theta_min': -2.35619, 'theta_max': 2.35619}
# Locked Calibration for the 270-Degree Encoder Servo (Servo 1)
CALIBRATION['j_thigh_lr'] = {'v_min': 0.084, 'v_max': 2.960, 'theta_min': -1.57079, 'theta_max': 1.57079} # UPDATED TO 180 DEG
# Locked Calibration for the 270-Degree Encoder Servo (Servo 2)
CALIBRATION['j_c1_lr'] = {'v_min': 0.083, 'v_max': 3.013, 'theta_min': -2.35619, 'theta_max': 2.35619}

# Update the replaced RR thigh to 180 degrees
CALIBRATION['j_thigh_rr']['theta_min'] = -1.57079
CALIBRATION['j_thigh_rr']['theta_max'] = 1.57079




def select_mux_channel(bus, channel):
    if USE_MUX:
        bus.write_byte(TCA_ADDR, 1 << channel)
        time.sleep(0.002)


def read_ads1115_raw(bus, ads_channel):
    config = (
        CONFIG_OS_SINGLE
        | MUX_MAP[ads_channel]
        | CONFIG_PGA_4_096V
        | CONFIG_MODE_SINGLE
        | CONFIG_DR_860SPS
        | CONFIG_COMP_DISABLE
    )
    hi = (config >> 8) & 0xFF
    lo = config & 0xFF
    bus.write_i2c_block_data(ADS_ADDR, ADS_REG_CONFIG, [hi, lo])
    time.sleep(0.0015)
    data = bus.read_i2c_block_data(ADS_ADDR, ADS_REG_CONVERSION, 2)
    raw = (data[0] << 8) | data[1]
    if raw > 32767:
        raw -= 65536
    return raw


def raw_to_voltage(raw):
    return raw * (GAIN_FSR / ADS_RESOLUTION)


def voltage_to_angle(voltage, joint_name):
    cal = CALIBRATION[joint_name]
    v_min, v_max = cal['v_min'], cal['v_max']
    t_min, t_max = cal['theta_min'], cal['theta_max']
    voltage = max(v_min, min(v_max, voltage))
    ratio = (voltage - v_min) / (v_max - v_min)
    return t_min + ratio * (t_max - t_min)


def read_all_joints(bus):
    result = {}
    if USE_MUX:
        for mux_ch, ads_channels in JOINT_MAP_MUX.items():
            select_mux_channel(bus, mux_ch)
            for ads_ch, joint_name in ads_channels.items():
                raw = read_ads1115_raw(bus, ads_ch)
                voltage = raw_to_voltage(raw)
                angle_rad = voltage_to_angle(voltage, joint_name)
                result[joint_name] = angle_rad
    else:
        # Read from single directly connected ADS1115
        for ads_ch, joint_name in JOINT_MAP_DIRECT.items():
            raw = read_ads1115_raw(bus, ads_ch)
            voltage = raw_to_voltage(raw)
            angle_rad = voltage_to_angle(voltage, joint_name)
            result[joint_name] = angle_rad
    return result


if __name__ == '__main__':
    bus = smbus2.SMBus(I2C_BUS)
    while True:
        joints = read_all_joints(bus)
        for name, angle in joints.items():
            print("{}: {:.3f} rad".format(name, angle))
        print("---")
        time.sleep(0.2)
