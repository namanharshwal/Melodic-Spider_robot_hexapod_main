import serial, time, smbus2, sys
import os
sys.path.append('src/phantomx_gazebo/scripts')
import servo_feedback_reader as sfr

bus = smbus2.SMBus(1)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def read_adcs():
    return [sfr.raw_to_voltage(sfr.read_ads1115_raw(bus, i)) for i in range(3)]

def move_servo(sid, pos):
    packet = bytearray([0x55, 0x55, 8, 0x03, 1, 0xC8, 0x00, sid, pos & 0xFF, (pos >> 8) & 0xFF])
    ser.write(packet)
    time.sleep(0.3)

print("Probing all 32 servos...")
base_adcs = read_adcs()
found = {0: None, 1: None, 2: None}

for sid in range(1, 33):
    move_servo(sid, 200)
    adcs_1 = read_adcs()
    move_servo(sid, 800)
    adcs_2 = read_adcs()
    move_servo(sid, 500)
    
    for ch in range(3):
        diff = abs(adcs_2[ch] - adcs_1[ch])
        if diff > 0.5:
            found[ch] = sid
            print("Found AIN{} mapped to Servo ID {}".format(ch, sid))

print("\nFinal Mapping:", found)
