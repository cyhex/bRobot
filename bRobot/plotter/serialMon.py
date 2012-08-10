import serial
import time

ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)

while ser:
    line = ser.readline()
    if line:
        print time.time(), line.strip()
