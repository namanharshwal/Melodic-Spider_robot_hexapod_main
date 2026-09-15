#!/usr/bin/env python
import sys
import time
import os
import smbus2

# Add script directory to path
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
import servo_feedback_reader as sfr

def main():
    print("====================================================")
    print("           Spider Servo Calibration Tool            ")
    print("====================================================")
    print("Reading analog voltages from AIN0 - AIN3...")
    print("Move the leg manually and watch the voltage ranges.")
    print("Press Ctrl+C to exit.")
    print("----------------------------------------------------")

    try:
        bus = smbus2.SMBus(sfr.I2C_BUS)
    except IOError as e:
        print("Error: Cannot open I2C bus {}. Verify permissions or wiring. ({})".format(sfr.I2C_BUS, e))
        sys.exit(1)

    while True:
        try:
            voltages = []
            for ch in range(4):
                raw = sfr.read_ads1115_raw(bus, ch)
                volts = sfr.raw_to_voltage(raw)
                voltages.append(volts)

            print("AIN0 (Coxa): {:.3f} V | AIN1 (Thigh): {:.3f} V | AIN2 (Tibia): {:.3f} V | AIN3 (LM Coxa): {:.3f} V".format(
                voltages[0], voltages[1], voltages[2], voltages[3]
            ))
            time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nCalibration tool exited.")
            break
        except Exception as e:
            print("Error reading sensors: {}".format(e))
            time.sleep(0.5)

if __name__ == '__main__':
    main()
