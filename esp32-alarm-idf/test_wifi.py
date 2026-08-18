#!/usr/bin/env python3
import serial
import time

print("ESP32-C3 WiFi Test Monitor")
print("Monitoring /dev/ttyACM0 at 115200 baud...")

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)
    
    print("\nDevice reset. Monitoring output...\n")
    
    start_time = time.time()
    while time.time() - start_time < 60:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            try:
                text = data.decode('utf-8', errors='replace')
                print(text, end='', flush=True)
            except:
                pass
        time.sleep(0.01)