#!/usr/bin/env python3
"""
RTS_viz.py

Real-time terminal visualization of the occupancy grid and IR sensor.
Robot is stationary at origin — no ESP32 pose integration yet.

Python formatting basics shown throughout this file:
  - Indentation (4 spaces) defines code blocks — there are no braces.
  - A function defined inside another function is invisible outside it.
  - Variables must be explicitly defined before use — no implicit zero.
  - 'global' keyword required when a nested function writes a module-level var.

Run from project root:
    python3 scripts/RTS_viz.py
"""

import serial, struct, time, threading, lgpio, os, re, sys, math

# =============================================================================
# LOAD CALIBRATION
# =============================================================================
def load_calibration(path='include/calibration.h'):
    if not os.path.exists(path):
        print(f"ERROR: {path} not found. Run RTS_calibrate.py first.")
        sys.exit(1)
    values = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'#define\s+(\w+)\s+(\d+)', line)
            if m:
                values[m.group(1)] = int(m.group(2))
    return values

cal    = load_calibration()
OFFSET = cal['LIDAR_FORWARD_OFFSET_DEG']

# =============================================================================
# GRID CONFIG
# =============================================================================
CELL_MM  = 20
GRID_W   = 100
GRID_H   = 100
ORIGIN_X = GRID_W // 2
ORIGIN_Y = GRID_H // 2

LOG_FREE = -1
LOG_OCC  =  2
LOG_MIN  = -10
LOG_MAX  =  10

grid = [[0] * GRID_W for _ in range(GRID_H)]

# =============================================================================
# COORDINATE HELPERS
# =============================================================================
def world_to_cell(x_mm, y_mm):
    col = ORIGIN_X + int(x_mm / CELL_MM)
    row = ORIGIN_Y - int(y_mm / CELL_MM)
    return col, row

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# =============================================================================
# BRESENHAM LINE ALGORITHM
# =============================================================================
def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

# =============================================================================
# GRID UPDATE
# =============================================================================
def update_grid(scan_data):
    rx, ry = ORIGIN_X, ORIGIN_Y
    for angle_deg, dist_mm in scan_data.items():
        if dist_mm <= 0:
            continue
        dist_mm = min(dist_mm, (GRID_W // 2 - 1) * CELL_MM)
        rad   = math.radians(angle_deg)
        ex_mm =  dist_mm * math.sin(rad)
        ey_mm =  dist_mm * math.cos(rad)
        ex, ey = world_to_cell(ex_mm, ey_mm)
        cells = list(bresenham(rx, ry, ex, ey))
        for i, (cx, cy) in enumerate(cells):
            if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                if i == len(cells) - 1:
                    grid[cy][cx] = clamp(grid[cy][cx] + LOG_OCC,  LOG_MIN, LOG_MAX)
                else:
                    grid[cy][cx] = clamp(grid[cy][cx] + LOG_FREE, LOG_MIN, LOG_MAX)

# =============================================================================
# SHARED STATE
# =============================================================================
latest_scan = {}
scan_lock   = threading.Lock()

latest_ir = None
ir_lock   = threading.Lock()

# =============================================================================
# LIDAR THREAD
# =============================================================================
def lidar_thread():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
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
                    raw   = round(start_a + (end_a - start_a) * i / 12.0)
                    robot = (raw - OFFSET) % 360
                    latest_scan[robot] = dist

# =============================================================================
# IR THREAD
# =============================================================================
def ir_thread(h, trig, echo):
    # 'global' tells Python we're writing to the module-level latest_ir,
    # not creating a new local variable with the same name inside this function.
    global latest_ir
    while True:
        lgpio.gpio_write(h, trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(h, trig, 0)

        deadline = time.time() + 0.04
        while lgpio.gpio_read(h, echo) == 0:
            if time.time() > deadline:
                with ir_lock:
                    latest_ir = None
                break
        else:
            start    = time.time()
            deadline = time.time() + 0.04
            while lgpio.gpio_read(h, echo) == 1:
                if time.time() > deadline:
                    with ir_lock:
                        latest_ir = None
                    break
            else:
                with ir_lock:
                    latest_ir = (time.time() - start) * 171500

        time.sleep(0.1)

# =============================================================================
# SAVE PGM
# Module-level function — NOT inside render(). Being at zero indentation means
# it exists in the module scope and can be called from anywhere in this file.
# If this were indented inside render(), the main loop could not see it.
# =============================================================================
def save_pgm():
    os.makedirs("maps", exist_ok=True)
    with open("maps/map.pgm", "w") as f:
        f.write(f"P2\n{GRID_W} {GRID_H}\n255\n")
        for row in range(GRID_H):
            for col in range(GRID_W):
                v = grid[row][col]
                if col == ORIGIN_X and row == ORIGIN_Y:
                    px = 128    # robot — mid gray
                elif v > 0:
                    px = 0      # occupied — black
                elif v < 0:
                    px = 255    # free — white
                else:
                    px = 180    # unknown — light gray
                f.write(f"{px} ")
            f.write("\n")

# =============================================================================
# RENDER
# Module-level function. Ends here — save_pgm() is separate, below.
# =============================================================================
IR_ALERT_MM = 300

def render():
    with scan_lock:
        snap = dict(latest_scan)
    update_grid(snap)

    lines = []
    lines.append("┌" + "─" * GRID_W + "┐")
    for row in range(GRID_H):
        line = "│"
        for col in range(GRID_W):
            if col == ORIGIN_X and row == ORIGIN_Y:
                line += "R"
            else:
                v = grid[row][col]
                if   v > 0: line += "█"
                elif v < 0: line += "·"
                else:       line += " "
        line += "│"
        lines.append(line)
    lines.append("└" + "─" * GRID_W + "┘")

    with ir_lock:
        ir = latest_ir

    if ir is not None:
        alert = "  ⚠ CLOSE" if ir < IR_ALERT_MM else ""
        lines.append(f"IR front: {ir:6.0f} mm{alert}")
    else:
        lines.append("IR front:  timeout")

    lines.append("ctrl+C to stop")

    print("\033[H\033[J", end="")
    print("\n".join(lines))

# =============================================================================
# INIT AND MAIN LOOP
# =============================================================================
h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, 17)
lgpio.gpio_claim_input(h, 27)

threading.Thread(target=lidar_thread, daemon=True).start()
threading.Thread(target=ir_thread, args=(h, 17, 27), daemon=True).start()

print("Warming up — 2 seconds...")
time.sleep(2)

# frame must be defined here before the loop — Python has no implicit zero.
# Without this line, 'frame += 1' inside the loop throws NameError.
frame = 0

try:
    while True:
        render()
        frame += 1
        # Save PGM every 10 frames (~2 seconds at 0.2s per frame).
        # The % operator gives the remainder of division — frame % 10 == 0
        # is true at frames 10, 20, 30... so save_pgm() runs every 10th frame.
        if frame % 10 == 0:
            save_pgm()
        time.sleep(0.2)   # single sleep at the end of the loop — not two

except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
    print("\nDone.")