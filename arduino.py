import serial
import time
import numpy as np

port = '/dev/ttyS0'

ard = serial.Serial(port,9600,timeout=5)

while True:
    val = np.cos(time.time()) * 3.14
    ard.write(val)
    time.sleep(5)