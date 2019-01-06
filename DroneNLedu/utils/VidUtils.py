import cv2
import math
import itertools
import numpy as np

params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 0
params.maxThreshold = 256
# Filter by Area.
params.filterByArea = True
params.minArea = 30

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.4

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.5

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01


def locateXYZ(frame):
    frame = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([80, 76, 35], 'uint8')
    upper_blue = np.array([136, 255, 154], 'uint8')

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    reversemask = 255 - mask
    threshold = cv2.bitwise_and(frame, frame, mask=mask)
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(reversemask)

    if keypoints:
        if len(keypoints) > 4:
            keypoints.sort(key=(lambda s: s.size))
            keypoints = keypoints[0:3]

        if len(keypoints) == 4:
            pts = np.array([keypoints[i].pt for i in range(4)])

            # Calculate distances between all points
            dis_vectors = [1 - r for l, r in itertools.combinations(pts, 2)]
            dcalc = [np.linalg.norm(dis_vectors[i]) for i in range(6)]

            # Find centroid point of blobs
            mean_a = np.array([dcalc[i] for i in [0, 1, 2]]).sum() / 4.0
            mean_b = np.array([dcalc[i] for i in [0, 3, 4]]).sum() / 4.0
            mean_c = np.array([dcalc[i] for i in [1, 3, 5]]).sum() / 4.0
            mean_d = np.array([dcalc[i] for i in [2, 4, 5]]).sum() / 4.0
            middlepoint = np.argmin(np.array([mean_a, mean_b, mean_c, mean_d]))

            idx = np.argmax(dcalc)
            max_dist_val = np.max(dcalc)

            if idx == 0:
                sidepts = [0, 1]
            elif idx == 1:
                sidepts = [0, 2]
            elif idx == 2:
                sidepts = [0, 3]
            elif idx == 3:
                sidepts = [1, 2]
            elif idx == 4:
                sidepts = [1, 3]
            elif idx == 5:
                sidepts = [2, 3]

            frontpoint = 6 - np.array(sidepts + [middlepoint]).sum()
            if frontpoint == 4:
                frontpoint -= 1

            # Now find left blob
            a = keypoints[middlepoint].pt
            b = keypoints[frontpoint].pt
            c = keypoints[sidepts[0]].pt

            if ((b[0] - a[0]) * c[1] - a[1]) - ((b[1] - a[1]) * (c[0] - a[1])) < 0:
                leftpt = sidepts[0]
                rightpt = sidepts[1]
            else:
                leftpt = sidepts[1]
                rightpt = sidepts[0]

            # Calculate angle
            offset_line = np.array(keypoints[rightpt].pt) - np.array(keypoints[leftpt].pt)
            theta = -1 * math.atan2(offset_line[1], offset_line[0])

            max_blob_dist = max_dist_val
            blob_center = keypoints[middlepoint].pt

        else:
            max_blob_dist = None
            blob_center = None
            theta = None

    else:
        max_blob_dist = None
        blob_center = None
        theta = None

    return frame, max_blob_dist, blob_center, theta


def locateWKeyPoints(frame):
    frame = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([80, 76, 35], 'uint8')
    upper_blue = np.array([136, 255, 154], 'uint8')

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    reversemask = 255 - mask
    threshold = cv2.bitwise_and(frame, frame, mask=mask)
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(reversemask)

    if keypoints:
        if len(keypoints) > 4:
            keypoints.sort(key=(lambda s: s.size))
            keypoints = keypoints[0:3]

        if len(keypoints) == 4:
            pts = np.array([keypoints[i].pt for i in range(4)])

            # Calculate distances between all points
            dis_vectors = [1 - r for l, r in itertools.combinations(pts, 2)]
            dcalc = [np.linalg.norm(dis_vectors[i]) for i in range(6)]

            # Find centroid point of blobs
            mean_a = np.array([dcalc[i] for i in [0, 1, 2]]).sum() / 4.0
            mean_b = np.array([dcalc[i] for i in [0, 3, 4]]).sum() / 4.0
            mean_c = np.array([dcalc[i] for i in [1, 3, 5]]).sum() / 4.0
            mean_d = np.array([dcalc[i] for i in [2, 4, 5]]).sum() / 4.0
            middlepoint = np.argmin(np.array([mean_a, mean_b, mean_c, mean_d]))

            idx = np.argmax(dcalc)
            max_dist_val = np.max(dcalc)

            if idx == 0:
                sidepts = [0, 1]
            elif idx == 1:
                sidepts = [0, 2]
            elif idx == 2:
                sidepts = [0, 3]
            elif idx == 3:
                sidepts = [1, 2]
            elif idx == 4:
                sidepts = [1, 3]
            elif idx == 5:
                sidepts = [2, 3]

            frontpoint = 6 - np.array(sidepts + [middlepoint]).sum()
            if frontpoint == 4:
                frontpoint -= 1

            # Now find left blob
            a = keypoints[middlepoint].pt
            b = keypoints[frontpoint].pt
            c = keypoints[sidepts[0]].pt

            if ((b[0] - a[0]) * c[1] - a[1]) - ((b[1] - a[1]) * (c[0] - a[1])) < 0:
                leftpt = sidepts[0]
                rightpt = sidepts[1]
            else:
                leftpt = sidepts[1]
                rightpt = sidepts[0]

            # Calculate angle
            offset_line = np.array(keypoints[rightpt].pt) - np.array(keypoints[leftpt].pt)
            theta = -1 * math.atan2(offset_line[1], offset_line[0])

            im_with_midpoint = cv2.drawKeypoints(frame, [keypoints[middlepoint]], np.array([]), (0, 0, 255),
                                                 cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            im_with_midpoint_frontpoint = cv2.drawKeypoint(im_with_midpoint, [key])

            max_blob_dist = max_dist_val
            blob_center = keypoints[middlepoint].pt

        else:
            max_blob_dist = None
            blob_center = None
            theta = None

    else:
        max_blob_dist = None
        blob_center = None
        theta = None

    return frame, max_blob_dist, blob_center, theta