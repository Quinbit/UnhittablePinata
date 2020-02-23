import numpy as np
import cv2
from time import time
import ipdb
import math
from numpy.linalg import lstsq
from numpy import ones,vstack
from sklearn.linear_model import LinearRegression

cap = cv2.VideoCapture(2)

prev = None

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if prev is None:
        prev = frame
    else:
        diff = cv2.absdiff(frame, prev)
        prev = frame
        mask2 = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
        cv2.imshow('frame',mask2)

        if abs(mask2.mean()) > 10:
            print("SWING {}".format(time()))
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break