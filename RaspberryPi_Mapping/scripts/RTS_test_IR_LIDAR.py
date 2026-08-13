#!/usr/bin/env python3
"""
RTS_test_IR_LiDAR.py
Run from project root: python3 scripts/RTS_test_IR_LiDAR.py
"""

import time, lgpio

IR_SENSORS = [
    {'name': '0°  fwd',   'trig': 17, 'echo': 27},
    {'name': '+45° RF',   'trig': 22, 'echo': 23},
    {'name': '-45° LF',   'trig': 24, 'echo': 25},
    {'name': '+90° R',    'trig': 5,  'echo': 6},
    {'name': '-90° L',    'trig': 12, 'echo': 13},
    {'name': '180° rear', 'trig': 16, 'echo': 20},
]

def read_ir(h, trig, echo):
    lgpio.gpio_write(h, trig, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, trig, 0)
    deadline = time.time() + 0.04
    while lgpio.gpio_read(h, echo) == 0:
        if time.time() > deadline: return None
    start = time.time()
    deadline = time.time() + 0.04
    while lgpio.gpio_read(h, echo) == 1:
        if time.time() > deadline: return None
    return (time.time() - start) * 171500

h = lgpio.gpiochip_open(4)
for s in IR_SENSORS:
    lgpio.gpio_claim_output(h, s['trig'])
    lgpio.gpio_claim_input(h, s['echo'])

print("ctrl+C to stop\n")
try:
    while True:
        readings = []
        for s in IR_SENSORS:
            d = read_ir(h, s['trig'], s['echo'])
            readings.append(f"{s['name']}: {int(d):5d}mm" if d else f"{s['name']}: ----  ")
        print("  |  ".join(readings))
        time.sleep(0.3)
except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
    print("\nDone.")
