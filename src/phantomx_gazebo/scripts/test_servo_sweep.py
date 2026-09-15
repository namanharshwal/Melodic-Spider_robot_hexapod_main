#!/usr/bin/env python
import serial
import time
import smbus2
import sys
import os

# Add script directory to path to import your I2C reader
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
import servo_feedback_reader as sfr

# Left Front Leg Servo IDs on LSC-32
SERVO_IDS = [8, 7, 6] # j_c1_lf, j_thigh_lf, j_tibia_lf

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
except Exception as e:
    print("Error: Could not open /dev/ttyUSB0. Is the bridge closed? ({})".format(e))
    sys.exit(1)

try:
    bus = smbus2.SMBus(sfr.I2C_BUS)
except Exception as e:
    print("Error: Could not open I2C bus. ({})".format(e))
    sys.exit(1)

def set_lsc(servo_ids, pos, move_time=1000):
    pos = max(0, min(1000, pos))
    count = len(servo_ids)
    length = count * 3 + 5
    packet = bytearray([0x55, 0x55, length, 0x03, count, move_time & 0xFF, (move_time >> 8) & 0xFF])
    for sid in servo_ids:
        packet.extend([sid, pos & 0xFF, (pos >> 8) & 0xFF])
    ser.write(packet)

def main():
    print("====================================================")
    print("       360-Degree Servo Voltage Sweep Test          ")
    print("====================================================")
    print("Sweeping servos 8, 7, 6 from LSC 0 to 1000...")
    
    # Optional: Start at 0 smoothly
    set_lsc(SERVO_IDS, 0, move_time=2000)
    time.sleep(2.5)
    
    for pos in range(0, 1001, 100):
        print("\n--> Commanding LSC Position: {} (approx {} deg)".format(pos, int(pos * 360.0 / 1000.0)))
        set_lsc(SERVO_IDS, pos, move_time=1000)
        time.sleep(1.5) # Wait for servos to reach position
        
        try:
            # Read all 3 ADC channels
            v_c1 = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 0))
            v_th = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 1))
            v_ti = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 2))
            print("    ADC Voltages: AIN0 (Coxa): {:.3f} V | AIN1 (Thigh): {:.3f} V | AIN2 (Tibia): {:.3f} V".format(v_c1, v_th, v_ti))
        except Exception as e:
            print("    Error reading ADC: {}".format(e))

    print("\nSweep Complete! Returning to Center (LSC 500)...")
    set_lsc(SERVO_IDS, 500, move_time=2000)
    time.sleep(2.0)
    ser.close()

if __name__ == '__main__':
    main()
