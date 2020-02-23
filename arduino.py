import serial
import time
import numpy as np

port = '/dev/ttyACM0'

ard = serial.Serial(port,9600,timeout=5)

while True:
    time.sleep(5)
    val = int(np.cos(time.time()) * 180 + 180)
    print(val)
    ard.write(str.encode(str(val)))
    # ard.write(str.encode('v 50'))
    # time.sleep(10)
    # ard.write(str.encode('c 50'))
    # time.sleep(10)