import cv2
import numpy as np
import math

def nothing(x):
    pass

def calibrate():

    cap = cv2.VideoCapture(0)

    ret, frame = cap.read()
    height, width = frame.shape[:2]

    frame_low = np.zeros((math.floor(height * 0.1), width, 3), np.uint8)
    frame_high = np.zeros((math.floor(height * 0.1), width, 3), np.uint8)

    while(True):

        ret, frame = cap.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_low = cv2.cvtColor(frame_low, cv2.COLOR_BGR2HSV)
        frame_high = cv2.cvtColor(frame_high, cv2.COLOR_BGR2HSV)

        hue_l = cv2.getTrackbarPos('Hue_L', 'Calib')
        hue_h = cv2.getTrackbarPos('Hue_H', 'Calib')

        sat_l = cv2.getTrackbarPos('Sat_L', 'Calib')
        sat_h = cv2.getTrackbarPos('Sat_H', 'Calib')

        bri_l = cv2.getTrackbarPos('Bri_L', 'Calib')
        bri_h = cv2.getTrackbarPos('Bri_H', 'Calib')

        lower = np.array([hue_l, sat_l, bri_l])
        higher = np.array([hue_h, sat_h, bri_h])

        frame_low[:] = lower
        frame_high[:] = higher

        font = cv2.FONT_HERSHEY_SIMPLEX

        frame_low = cv2.cvtColor(frame_low, cv2.COLOR_HSV2BGR)
        f_height, f_width = frame_low.shape[:2]
        cv2.putText(frame_low, 'LOW', (5, f_height - 5), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame_low, 'LOW', (5, f_height - 5), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        frame_high = cv2.cvtColor(frame_high, cv2.COLOR_HSV2BGR)
        cv2.putText(frame_high, 'HIGH', (5, f_height - 5), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame_high, 'HIGH', (5, f_height - 5), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        mask = cv2.inRange(hsv, lower, higher)
        mask = cv2.erode(mask, kernel, iterations = 2)
        mask = cv2.dilate(mask, kernel, iterations = 2)

        res = cv2.bitwise_and(frame, frame, mask= mask)

        output = np.concatenate((frame, res), axis = 1)
        aux = np.concatenate((frame_low, frame_high), axis = 1)
        output = np.concatenate((aux, output), axis = 0)

        cv2.imshow('Calib', output)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

kernel = np.ones((3,3), np.uint8)

cv2.namedWindow('Calib')

cv2.createTrackbar('Hue_L', 'Calib', 0, 180, nothing)
cv2.createTrackbar('Hue_H', 'Calib', 0, 180, nothing)

cv2.createTrackbar('Sat_L', 'Calib', 0, 255, nothing)
cv2.createTrackbar('Sat_H', 'Calib', 0, 255, nothing)

cv2.createTrackbar('Bri_L', 'Calib', 0, 255, nothing)
cv2.createTrackbar('Bri_H', 'Calib', 0, 255, nothing)

calibrate()
cap.release()
cv2.destroyAllWindows()
