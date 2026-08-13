import serial, struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
print("Reading LDS-02 — ctrl+C to stop")

while True:
    byte = ser.read(1)
    if byte != b'\x54':
        continue
    rest = ser.read(46)
    if len(rest) < 46 or rest[0] != 0x2C:
        continue
    pkt = byte + rest

    start_angle = struct.unpack_from('<H', pkt, 4)[0] / 100.0
    end_angle   = struct.unpack_from('<H', pkt, 42)[0] / 100.0

    print(f"\nAngles {start_angle:.1f}° → {end_angle:.1f}°")
    for i in range(12):
        dist = struct.unpack_from('<H', pkt, 6 + i*3)[0]
        angle = start_angle + (end_angle - start_angle) * i / 12
        if dist > 0:
            print(f"  {angle:6.1f}°  {dist}mm")
