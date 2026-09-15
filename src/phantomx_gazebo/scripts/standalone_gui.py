#!/usr/bin/env python
try:
    import Tkinter as tk
    import ttk
except ImportError:
    import tkinter as tk
    from tkinter import ttk
import serial
import smbus2
import sys
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
import servo_feedback_reader as sfr

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
except Exception as e:
    print("Serial error: {}. Make sure ROS launch is closed so the port is free!".format(e))
    sys.exit(1)

try:
    bus = smbus2.SMBus(sfr.I2C_BUS)
except Exception as e:
    print("I2C error: {}".format(e))
    bus = None

def set_lsc(servo_id, pos):
    move_time = 150
    # Pack a direct move command to LSC-32
    packet = bytearray([0x55, 0x55, 8, 0x03, 1, move_time & 0xFF, (move_time >> 8) & 0xFF, servo_id, pos & 0xFF, (pos >> 8) & 0xFF])
    try:
        ser.write(packet)
    except:
        pass

def on_slider_move(val):
    try:
        sid = int(id_var.get())
        pos = int(val)
        set_lsc(sid, pos)
    except:
        pass

import json
def update_adc():
    if bus:
        try:
            v0 = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 0))
            v1 = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 1))
            v2 = sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, 2))
            adc_label.config(text="Live ADC: AIN0: {:.3f}V | AIN1: {:.3f}V | AIN2: {:.3f}V".format(v0, v1, v2))
            
            # Log for backend verification
            with open("/tmp/servo_calibration.log", "a") as f:
                current_pos = slider.get() if 'slider' in globals() else 1500
                f.write(json.dumps({"pos": current_pos, "v0": v0, "v1": v1, "v2": v2}) + "\n")
        except:
            adc_label.config(text="ADC Read Error (Check Wiring)")
    
    # Loop every 200ms
    root.after(200, update_adc)

root = tk.Tk()
root.title("360-Degree Direct Hardware Tester")
root.geometry("500x250")

tk.Label(root, text="Select Servo ID (0-32):", font=("Arial", 12)).pack(pady=(15, 5))
id_var = tk.StringVar(value="8")
entry = tk.Entry(root, textvariable=id_var, justify='center', font=("Arial", 14), width=5)
entry.pack()

tk.Label(root, text="Direct PWM Position (500 - 2500 us):", font=("Arial", 12)).pack(pady=(15, 5))
slider = tk.Scale(root, from_=500, to=2500, orient='horizontal', length=400, command=on_slider_move)
slider.set(1500)
slider.pack()

adc_label = tk.Label(root, text="Live ADC: Waiting...", font=("Arial", 14, "bold"), fg="blue")
adc_label.pack(pady=20)

update_adc()
root.mainloop()
