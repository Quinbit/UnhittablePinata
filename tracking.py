import numpy as np
import cv2
from time import time
import ipdb
import math
from numpy.linalg import lstsq
from numpy import ones,vstack

cap = cv2.VideoCapture(2)
cap0 = cv2.VideoCapture(0)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
start_time = time()

overlapThresh = 0.1
calibration_size = 30
calibration_delta = np.array([20,50,50])
min_size = 0
countdown = 3

# def non_max_suppression_fast(boxes, overlapThresh):
# 	# if there are no boxes, return an empty list
#     if len(boxes) == 0:
#         return []
# 	# if the bounding boxes integers, convert them to floats --
# 	# this is important since we'll be doing a bunch of divisions
#     if boxes.dtype.kind == "i":
#         boxes = boxes.astype("float")
# 	# initialize the list of picked indexes	
#     pick = []
# 	# grab the coordinates of the bounding boxes
#     x1 = boxes[:,0]
#     y1 = boxes[:,1]
#     x2 = boxes[:,2]
#     y2 = boxes[:,3]
# 	# compute the area of the bounding boxes and sort the bounding
# 	# boxes by the bottom-right y-coordinate of the bounding box
#     area = (x2 - x1 + 1) * (y2 - y1 + 1)
#     idxs = np.argsort(y2)
# 	# keep looping while some indexes still remain in the indexes
# 	# list
#     while len(idxs) > 0:
# 		# grab the last index in the indexes list and add the
# 		# index value to the list of picked indexes
#         last = len(idxs) - 1
#         i = idxs[last]
#         pick.append(i)
# 		# find the largest (x, y) coordinates for the start of
# 		# the bounding box and the smallest (x, y) coordinates
# 		# for the end of the bounding box
#         xx1 = np.maximum(x1[i], x1[idxs[:last]])
#         yy1 = np.maximum(y1[i], y1[idxs[:last]])
#         xx2 = np.minimum(x2[i], x2[idxs[:last]])
#         yy2 = np.minimum(y2[i], y2[idxs[:last]])
# 		# compute the width and height of the bounding box
#         w = np.maximum(0, np.abs(xx2 - xx1) + 1)
#         h = np.maximum(0, np.abs(yy2 - yy1) + 1)
# 		# compute the ratio of overlap
#         overlap = (w * h) / area[idxs[:last]]
# 		# delete all indexes from the index list that have
#         idxs = np.delete(idxs, np.concatenate(([last],
#             np.where(overlap > overlapThresh)[0])))
# 	# return only the bounding boxes that were picked using the
# 	# integer data type

#     for p in range(len(pick)-1,-1,-1):
#         x1 = boxes[pick[p],0]
#         y1 = boxes[pick[p],1]
#         x2 = boxes[pick[p],2]
#         y2 = boxes[pick[p],3]
#         if (x2 - x1) * (y2 - y1) < min_size:
#             del pick[p]
#     return boxes[pick].astype("int")

def draw_center(img, color=(255,12,12)):
    center_x = img.shape[1] // 2
    center_y = img.shape[0] // 2

    cv2.rectangle(img, (center_x - calibration_size, center_y - calibration_size), (center_x + calibration_size, center_y + calibration_size), color, 2)

    return center_x, center_y

def in_bounds(size, point):
    return point[0] >= 0 and point[0] <= size[0] and point[1] >= 0 and point[1] <= size[1]

upper = None
lower = None
means = []

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # ret, frame0 = cap0.read()
    # frame = np.concatenate((frame, frame), 1)
    img = frame.copy()

    if time() - start_time < countdown:
        draw_center(img)
        img = cv2.flip(img, 1)
        cv2.putText(img, "Calibrating in {}".format(1+int(countdown + start_time - time())), (50,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,200), thickness=4)
        img = cv2.flip(img, 1)
        mask = img
        pass
    else:
        if time() - start_time < 6:
            x, y = draw_center(img)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            means.append(hsv[y-calibration_size:y+calibration_size,x-calibration_size:x+calibration_size].mean(0).mean(0))
            lower = np.array([math.inf, math.inf, math.inf])
            upper = np.array([0,0,0])

            for m in means:
                lower = np.minimum(lower, m - calibration_delta)
                upper = np.maximum(upper, m + calibration_delta)

            mask = img

            print(lower)
            print(upper)
        else:
            # Our operations on the frame come here
            # lower = np.array([50, 50, 50], dtype="uint8")
            # upper = np.array([70, 100, 100], dtype="uint8")
            # import ipdb; ipdb.set_trace()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            cv2.erode(mask, None, iterations=6)
            cv2.dilate(mask, None, iterations=8)

            x, y = np.nonzero(mask)
            # import ipdb; ipdb.set_trace()

            if len(x) > 0:
                A = vstack([x,ones(len(x))]).T
                m, c = lstsq(A, y)[0]

                h = img.shape[0]
                w = img.shape[1]

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
                    cv2.line(img, points[0], points[1], (255,0,0),5)

            #########

            # rho = 1  # distance resolution in pixels of the Hough grid
            # theta = np.pi / 180  # angular resolution in radians of the Hough grid
            # threshold = 15  # minimum number of votes (intersections in Hough grid cell)
            # min_line_length = 50  # minimum number of pixels making up a line
            # max_line_gap = 20  # maximum gap in pixels between connectable line segments
            # line_image = np.copy(img) * 0  # creating a blank to draw lines on

            # lines = cv2.HoughLinesP(mask, rho, theta, threshold, np.array([]),
            #         min_line_length, max_line_gap)

            # # import ipdb; ipdb.set_trace()

            # if lines is not None:
            #     # lines = np.expand_dims(non_max_suppression_fast(lines.squeeze(1), overlapThresh), 1)

            #     for line in lines:
            #         for x1,y1,x2,y2 in line:
            #             cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)
            

    # Display the resulting frame
    cv2.imshow('frame',cv2.flip(img, 1))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('c'):
        means = []
        start_time = time()

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()