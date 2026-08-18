import serial
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30

s = serial.Serial()
s.port = port
s.baudrate = 115200
s.dtr = False
s.rts = False
s.timeout = 0.5
s.open()
s.dtr = False
s.rts = False

end = time.time() + duration
while time.time() < end:
    line = s.readline()
    if line:
        print(line.decode(errors="replace").rstrip())
s.close()
