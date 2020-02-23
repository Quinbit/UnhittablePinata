import numpy as np
import cv2
from time import time
import ipdb
import math
from numpy.linalg import lstsq
from numpy import ones,vstack
from sklearn.linear_model import LinearRegression

cap = cv2.VideoCapture(2)
cap0 = cv2.VideoCapture(0)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
start_time = time()

overlapThresh = 0.1
calibration_size = 50
calibration_delta_min = np.array([20,20,20])
calibration_delta_max = np.array([20,20,20])
min_size = 0
countdown = 3

def draw_center(img, color=(255,12,12)):
    center_x = img.shape[1] // 2
    center_y = img.shape[0] // 2

    cv2.rectangle(img, (center_x - calibration_size, center_y - calibration_size), (center_x + calibration_size, center_y + calibration_size), color, 2)

    return center_x, center_y

def in_bounds(size, point):
    return point[0] >= 0 and point[0] <= size[1] and point[1] >= 0 and point[1] <= size[0]

def circle_intersect(m, constant, x, y, r):
    a = (1 + m**2)
    b = (-2 * x + 2*(constant - y) * m)
    c = (-x ** 2 + (constant - y) ** 2 - r**2)
    # ipdb.set_trace()

    factor = b**2 - 4 * a * c

    if factor < 0:
        return None, None
    else:
        x1 = (-b + (factor) ** 0.5) / (2 * a)
        x2 = (-b - (factor) ** 0.5) / (2 * a)
        y1 = x1 * m + constant
        y2 = x2 * m + constant

    return (x1, y1), (x2, y2)

def get_angle(x1, y1, x2, y2, cx, cy):
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    delta_x = cx - mid_x
    delta_y = cy - mid_y

    return np.angle(delta_x + 1j * delta_y)

upper = None
lower = None
means = []
background = None
prev = None
past_angles = []
past_timesteps = 10

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
            if len(past_angles) > 0:
                print("SWING {}".format(past_angles[0]))
                #Serial print
                del past_angles[-1]

    # ret, frame0 = cap0.read()
    # frame = np.concatenate((frame, frame), 1)
    img = frame.copy()
    if time() - start_time < countdown:
        background = frame
        img = frame
        mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    elif time() - start_time < countdown + 5:
        draw_center(img)
        img = cv2.flip(img, 1)
        cv2.putText(img, "Calibrating in {}".format(int(5 + countdown + start_time - time())), (50,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,200), thickness=4)
        img = cv2.flip(img, 1)
        mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    elif time() - start_time < countdown + 7:
        x, y = draw_center(img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        means.append(hsv[y-calibration_size:y+calibration_size,x-calibration_size:x+calibration_size].mean(0).mean(0))
        # cv2.putText(img, "Calibrating to {}".format(means[-1]), (50,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,200), thickness=4)
        lower = np.array([math.inf, math.inf, math.inf])
        upper = np.array([0,0,0])

        for m in means:
            lower = np.minimum(lower, m - calibration_delta_min)
            upper = np.maximum(upper, m + calibration_delta_max)

        mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print(means[-1])
    else:
        # Our operations on the frame come here
        # lower = np.array([30, 25, 240], dtype="uint8")
        # upper = np.array([40, 50, 255], dtype="uint8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        diff = cv2.absdiff(img, background)
        mask2 = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
        # import ipdb; ipdb.set_trace()
        mask = cv2.bitwise_and(mask, mask2[:,:,0])
        cv2.erode(mask, None, iterations=8)
        cv2.dilate(mask, None, iterations=10)

        x, y = np.nonzero(mask)

        if len(x) > 0:
            A = vstack([x,ones(len(x))]).T
            m, c = lstsq(A, y)[0]

            # print(m)

            w = img.shape[0]
            h = img.shape[1]

            intersection_t = (0, int(-c / m))
            intersection_b = (int(h), int((h - c) / m))
            intersection_r = (int(c), 0)
            intersection_l = (int(w * m + c), int(w))

            points = []

            if in_bounds(mask.shape, intersection_t):
                points.append(intersection_t)
            if in_bounds(mask.shape, intersection_b):
                points.append(intersection_b)
            if in_bounds(mask.shape, intersection_r):
                points.append(intersection_r)
            if in_bounds(mask.shape, intersection_l):
                points.append(intersection_l)

            if len(points) == 2:
                # print(points)
                cv2.line(img, points[0], points[1], (255,0,0),5)
            else:
                print("OOB", mask.shape, intersection_t, intersection_r, intersection_l, intersection_b)

            center_x = img.shape[1] // 2
            center_y = img.shape[0] // 2

            radius = 200

            p1, p2 = circle_intersect(m, c, center_y, center_x, 200)

            if p1 is None:
                continue
            angle = get_angle(*p1, *p2, center_y, center_x)

            past_angles.append(angle)
            if len(past_angles) > past_timesteps:
                del past_angles[0]

            p1 = (int(p1[0]), int(p1[1]))
            p2 = (int(p2[0]), int(p2[1]))

            img = cv2.circle(img, p1, 2, color=(255,0,0))
            img = cv2.circle(img, p2, 2, color=(255,0,0))
            # print(p1, p2)

            img = cv2.circle(img, (center_x, center_y), radius, color=(0,0,255))
            point = (int(center_x + np.sin(angle) * radius), int(center_y + np.cos(angle) * radius))
            img = cv2.arrowedLine(img, (center_x, center_y), point, color=(0,255,0)) 
            

    # Display the resulting frame
    cv2.imshow('frame',cv2.flip(np.concatenate((np.repeat(np.expand_dims(mask,-1), 3, -1), img),1), 1))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('c'):
        means = []
        start_time = time()

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()