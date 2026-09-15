#!/usr/bin/env python3
import time
import sys
import os
import smbus2

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
import servo_feedback_reader as sfr

def main():
    print("====================================================")
    print("      AUTOMATED SPIDER SERVO CALIBRATION TOOL       ")
    print("====================================================")
    print("Listening to AIN0-AIN3 for 15 seconds...")
    print(">>> MOVE THE LEFT FRONT LEG TO ITS PHYSICAL LIMITS NOW! <<<")
    
    try:
        bus = smbus2.SMBus(sfr.I2C_BUS)
    except IOError as e:
        print("Error: Cannot open I2C bus. Verify permissions or wiring. ({})".format(e))
        sys.exit(1)

    min_volts = [999.0, 999.0, 999.0, 999.0]
    max_volts = [-999.0, -999.0, -999.0, -999.0]
    
    start_time = time.time()
    
    while time.time() - start_time < 15.0:
        for ch in range(4):
            try:
                raw = sfr.read_ads1115_raw(bus, ch)
                volts = sfr.raw_to_voltage(raw)
                
                if volts < min_volts[ch]:
                    min_volts[ch] = volts
                if volts > max_volts[ch]:
                    max_volts[ch] = volts
            except Exception:
                pass
        time.sleep(0.05)

    print("\nCalibration Complete!")
    print("--- RESULTS ---")
    print("AIN0 (j_c1_lf):    min = {:.3f} V, max = {:.3f} V".format(min_volts[0], max_volts[0]))
    print("AIN1 (j_thigh_lf): min = {:.3f} V, max = {:.3f} V".format(min_volts[1], max_volts[1]))
    print("AIN2 (j_tibia_lf): min = {:.3f} V, max = {:.3f} V".format(min_volts[2], max_volts[2]))
    print("AIN3 (j_c1_lm):    min = {:.3f} V, max = {:.3f} V".format(min_volts[3], max_volts[3]))
    print("---------------")

if __name__ == '__main__':
    main()
