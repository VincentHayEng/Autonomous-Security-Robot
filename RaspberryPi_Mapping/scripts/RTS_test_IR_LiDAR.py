#!/usr/bin/env python3
"""
test_combined.py

Reads include/calibration.h at startup to find the LiDAR forward offset.
This is the first example of the two-phase pattern the full project uses:
  Phase 1 — calibrate.py runs once, writes shared config.
  Phase 2 — runtime programs read that config and go.
"""

import serial, struct, time, threading, lgpio, re, sys, os

# =============================================================================
# LOAD CALIBRATION
# =============================================================================
# Parse #define lines from the C header. re.match captures name and value.
# This same file will be #included by C programs later — one source of truth.

def load_calibration(path='include/calibration.h'):
    if not os.path.exists(path):
        print(f"ERROR: {path} not found. Run calibrate.py first.")
        sys.exit(1)
    values = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'#define\s+(\w+)\s+(\d+)', line)
            if m:
                values[m.group(1)] = int(m.group(2))
    return values

cal     = load_calibration()
OFFSET  = cal['LIDAR_FORWARD_OFFSET_DEG']   # Physical forward in LiDAR-space
WINDOW  = cal['LIDAR_FORWARD_WINDOW_DEG']   # ± this many degrees = "front"

print(f"Calibration loaded — forward = {OFFSET}°, window = ±{WINDOW}°\n")

# =============================================================================
# CONFIG
# =============================================================================
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

IR_SENSORS = [
    {'name': 'front (0°)',   'trig': 17, 'echo': 27},
    # {'name': 'right (+45°)', 'trig': 22, 'echo': 23},
    # {'name': 'left  (-45°)', 'trig': 24, 'echo': 25},
]

# =============================================================================
# SHARED LIDAR STATE  (unchanged from before)
# =============================================================================
latest_scan = {}
scan_lock   = threading.Lock()

def lidar_thread():
    ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=2)
    while True:
        if ser.read(1) != b'\x54':
            continue
        rest = ser.read(46)
        if len(rest) < 46 or rest[0] != 0x2C:
            continue
        pkt     = b'\x54' + rest
        start_a = struct.unpack_from('<H', pkt,  4)[0] / 100.0
        end_a   = struct.unpack_from('<H', pkt, 42)[0] / 100.0
        with scan_lock:
            for i in range(12):
                dist = struct.unpack_from('<H', pkt, 6 + i * 3)[0]
                if dist > 0:
                    angle = round(start_a + (end_a - start_a) * i / 12.0)
                    latest_scan[angle] = dist

# =============================================================================
# IR SENSOR  (unchanged)
# =============================================================================
def read_ir(h, trig, echo):
    lgpio.gpio_write(h, trig, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, trig, 0)
    deadline = time.time() + 0.04
    while lgpio.gpio_read(h, echo) == 0:
        if time.time() > deadline:
            return None
    start    = time.time()
    deadline = time.time() + 0.04
    while lgpio.gpio_read(h, echo) == 1:
        if time.time() > deadline:
            return None
    return (time.time() - start) * 171500

# =============================================================================
# INIT
# =============================================================================
h = lgpio.gpiochip_open(4)
for s in IR_SENSORS:
    lgpio.gpio_claim_output(h, s['trig'])
    lgpio.gpio_claim_input(h, s['echo'])

threading.Thread(target=lidar_thread, daemon=True).start()
print("Combined sensor test — ctrl+C to stop\n")

# =============================================================================
# MAIN LOOP
# =============================================================================
try:
    while True:
        for s in IR_SENSORS:
            d = read_ir(h, s['trig'], s['echo'])
            if d is not None:
                print(f"  IR {s['name']:15s}: {d:7.0f} mm")
            else:
                print(f"  IR {s['name']:15s}: timeout")

        # Forward slice using calibrated offset.
        # We wrap around 360° by normalising every angle relative to OFFSET,
        # then checking if it falls within ±WINDOW of zero.
        with scan_lock:
            fwd = {}
            for a, d in latest_scan.items():
                # Shift the angle so OFFSET maps to 0, then wrap to -180..180.
                relative = (a - OFFSET + 180) % 360 - 180
                if abs(relative) <= WINDOW:
                    fwd[a] = d

        if fwd:
            avg = sum(fwd.values()) / len(fwd)
            print(f"  LiDAR forward:      {avg:7.0f} mm  ({len(fwd)} samples)")
        else:
            print(f"  LiDAR forward:      no data yet")

        print("  ---")
        time.sleep(0.5)

except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
    print("\nDone.")
