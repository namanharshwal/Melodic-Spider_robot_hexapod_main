#!/usr/bin/env python3
import rospy
import serial
import time
import tkinter as tk
from tkinter import ttk

UART_PORT = '/dev/ttyUSB0'
UART_BAUD = 9600

SERVO_MAP = [
    ('j_c1_lf', 8), ('j_thigh_lf', 7), ('j_tibia_lf', 6),
    ('j_c1_lm', 5), ('j_thigh_lm', 4), ('j_tibia_lm', 3),
    ('j_c1_lr', 2), ('j_thigh_lr', 1), ('j_tibia_lr', 0),
    ('j_c1_rf', 22), ('j_thigh_rf', 23), ('j_tibia_rf', 24),
    ('j_c1_rm', 25), ('j_thigh_rm', 26), ('j_tibia_rm', 27),
    ('j_c1_rr', 28), ('j_thigh_rr', 29), ('j_tibia_rr', 30),
]

SAFE_START = {
    'j_c1_lf': 500, 'j_thigh_lf': 500, 'j_tibia_lf': 500,
    'j_c1_lm': 500, 'j_thigh_lm': 500, 'j_tibia_lm': 500,
    'j_c1_lr': 500, 'j_thigh_lr': 500, 'j_tibia_lr': 500,
    'j_c1_rf': 500, 'j_thigh_rf': 500, 'j_tibia_rf': 500,
    'j_c1_rm': 500, 'j_thigh_rm': 500, 'j_tibia_rm': 500,
    'j_c1_rr': 500, 'j_thigh_rr': 500, 'j_tibia_rr': 500,
}

MOVE_TIME = 300

class ServoTester:
    def __init__(self):
        rospy.init_node('servo_slider_tester', anonymous=True)
        self.ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        time.sleep(1.0)
        self.root = tk.Tk()
        self.root.title('Spider Robot Servo Slider Tester')
        self.root.geometry('950x900')
        self.status = tk.StringVar(value='Connected to LSC-32 @ 9600')
        self.sliders = {}
        self.labels = {}
        self.build_ui()
        self.center_all()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    def send_servo(self, servo_id, position, move_time=MOVE_TIME):
        position = max(0, min(1000, int(position)))
        pkt = bytes([
            0x55, 0x55,
            0x08,
            0x03,
            0x01,
            move_time & 0xFF,
            (move_time >> 8) & 0xFF,
            servo_id,
            position & 0xFF,
            (position >> 8) & 0xFF
        ])
        self.ser.write(pkt)

    def build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill='x')

        ttk.Label(top, text='Spider Robot Servo Tester', font=('Arial', 16, 'bold')).pack(anchor='w')
        ttk.Label(top, text='Use sliders to test each servo one by one. Safe range is 0 to 1000; start near 500.', foreground='blue').pack(anchor='w', pady=(3, 10))

        btns = ttk.Frame(top)
        btns.pack(fill='x', pady=(0, 10))

        ttk.Button(btns, text='Center All (500)', command=self.center_all).pack(side='left', padx=4)
        ttk.Button(btns, text='Stand Pose', command=self.stand_pose).pack(side='left', padx=4)
        ttk.Button(btns, text='Relax Test Pose', command=self.relax_pose).pack(side='left', padx=4)

        body = ttk.Frame(self.root, padding=10)
        body.pack(fill='both', expand=True)

        canvas = tk.Canvas(body)
        scrollbar = ttk.Scrollbar(body, orient='vertical', command=canvas.yview)
        scrollable = ttk.Frame(canvas)

        scrollable.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))

        canvas.create_window((0, 0), window=scrollable, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        for idx, (joint, sid) in enumerate(SERVO_MAP):
            frm = ttk.LabelFrame(scrollable, text=f'{joint}   [Servo ID {sid}]', padding=8)
            frm.grid(row=idx, column=0, sticky='ew', padx=5, pady=4)
            frm.columnconfigure(1, weight=1)

            self.labels[joint] = ttk.Label(frm, text=str(SAFE_START[joint]), width=6)
            self.labels[joint].grid(row=0, column=2, padx=6)

            scale = tk.Scale(frm, from_=0, to=1000, orient='horizontal', resolution=1,
                             command=lambda v, j=joint, s=sid: self.on_slide(j, s, v), length=620)
            scale.set(SAFE_START[joint])
            scale.grid(row=0, column=1, sticky='ew')

            ttk.Button(frm, text='Send', command=lambda j=joint, s=sid, sc=scale: self.manual_send(j, s, sc)).grid(row=0, column=3, padx=6)
            ttk.Button(frm, text='500', command=lambda j=joint, s=sid, sc=scale: self.set_servo(sc, j, s, 500)).grid(row=0, column=4, padx=2)
            ttk.Button(frm, text='450', command=lambda j=joint, s=sid, sc=scale: self.set_servo(sc, j, s, 450)).grid(row=0, column=5, padx=2)
            ttk.Button(frm, text='550', command=lambda j=joint, s=sid, sc=scale: self.set_servo(sc, j, s, 550)).grid(row=0, column=6, padx=2)

            self.sliders[joint] = scale

        bottom = ttk.Frame(self.root, padding=10)
        bottom.pack(fill='x')
        ttk.Label(bottom, textvariable=self.status, foreground='green').pack(anchor='w')
        ttk.Label(bottom, text='Close the window to recenter all servos automatically.', foreground='red').pack(anchor='w')

    def on_slide(self, joint, servo_id, value):
        value = int(float(value))
        self.labels[joint].config(text=str(value))
        self.send_servo(servo_id, value)
        self.status.set(f'Sent {joint} (ID {servo_id}) -> {value}')

    def manual_send(self, joint, servo_id, slider):
        value = int(slider.get())
        self.send_servo(servo_id, value)
        self.labels[joint].config(text=str(value))
        self.status.set(f'Manual send {joint} (ID {servo_id}) -> {value}')

    def set_servo(self, slider, joint, servo_id, value):
        slider.set(value)
        self.send_servo(servo_id, value)
        self.labels[joint].config(text=str(value))
        self.status.set(f'Set {joint} (ID {servo_id}) -> {value}')

    def center_all(self):
        for joint, sid in SERVO_MAP:
            self.send_servo(sid, 500, 600)
            if joint in self.sliders:
                self.sliders[joint].set(500)
            if joint in self.labels:
                self.labels[joint].config(text='500')
            time.sleep(0.03)
        self.status.set('All servos centered to 500')

    def stand_pose(self):
        pose = {
            'j_c1_lf': 500, 'j_thigh_lf': 560, 'j_tibia_lf': 430,
            'j_c1_lm': 500, 'j_thigh_lm': 560, 'j_tibia_lm': 430,
            'j_c1_lr': 500, 'j_thigh_lr': 560, 'j_tibia_lr': 430,
            'j_c1_rf': 500, 'j_thigh_rf': 440, 'j_tibia_rf': 570,
            'j_c1_rm': 500, 'j_thigh_rm': 440, 'j_tibia_rm': 570,
            'j_c1_rr': 500, 'j_thigh_rr': 440, 'j_tibia_rr': 570,
        }
        for joint, sid in SERVO_MAP:
            value = pose[joint]
            self.send_servo(sid, value, 700)
            self.sliders[joint].set(value)
            self.labels[joint].config(text=str(value))
            time.sleep(0.03)
        self.status.set('Stand pose sent')

    def relax_pose(self):
        pose = {
            'j_c1_lf': 500, 'j_thigh_lf': 520, 'j_tibia_lf': 480,
            'j_c1_lm': 500, 'j_thigh_lm': 520, 'j_tibia_lm': 480,
            'j_c1_lr': 500, 'j_thigh_lr': 520, 'j_tibia_lr': 480,
            'j_c1_rf': 500, 'j_thigh_rf': 480, 'j_tibia_rf': 520,
            'j_c1_rm': 500, 'j_thigh_rm': 480, 'j_tibia_rm': 520,
            'j_c1_rr': 500, 'j_thigh_rr': 480, 'j_tibia_rr': 520,
        }
        for joint, sid in SERVO_MAP:
            value = pose[joint]
            self.send_servo(sid, value, 700)
            self.sliders[joint].set(value)
            self.labels[joint].config(text=str(value))
            time.sleep(0.03)
        self.status.set('Relax pose sent')

    def on_close(self):
        try:
            self.center_all()
            time.sleep(0.8)
            self.ser.close()
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    ServoTester().run()
