#!/usr/bin/env python

import ta_vision

# import color_detection
from vision.camera import Camera

import cv2 as cv
import numpy as np
import time

tuner_window = "Threshold Calibration"
basic_window = "Raw Frame"

def setup(cb=None, lower=(0, 0, 0), upper=(180, 255, 255)):
    if cb is None:
        cb = on_change
    cv.namedWindow(tuner_window)
    cv.createTrackbar('Hue_Lo', tuner_window, lower[0], 180, cb)
    cv.createTrackbar('Hue_Hi', tuner_window, upper[0], 180, cb)
    cv.createTrackbar('Sat_Lo', tuner_window, lower[1], 255, cb)
    cv.createTrackbar('Sat_Hi', tuner_window, upper[1], 255, cb)
    cv.createTrackbar('Val_Lo', tuner_window, lower[2], 255, cb)
    cv.createTrackbar('Val_Hi', tuner_window, upper[2], 255, cb)

def on_change(x):
    pass

def get_value():
    hue_lo = cv.getTrackbarPos('Hue_Lo', tuner_window)
    hue_hi = cv.getTrackbarPos('Hue_Hi', tuner_window)
    sat_lo = cv.getTrackbarPos('Sat_Lo', tuner_window)
    sat_hi = cv.getTrackbarPos('Sat_Hi', tuner_window)
    val_lo = cv.getTrackbarPos('Val_Lo', tuner_window)
    val_hi = cv.getTrackbarPos('Val_Hi', tuner_window)
    return (
        np.array([hue_lo, sat_lo, val_lo]),
        np.array([hue_hi, sat_hi, val_hi])
    )

if __name__ == "__main__":
    cap = Camera(port=5600)

    print("Wait for camera capture..")
    frame = cap.capture()
    while frame is None:
        time.sleep(0.05)
        frame = cap.capture()
    print("First frame captured!")
    
    setup()
    cv.imshow(tuner_window, frame)
    cv.waitKey(15)

    while True:
        frame = cap.capture()
        lower, upper = get_value()
        mask = cv.inRange(frame, lower, upper)
        
        cv.imshow(basic_window, frame)
        cv.imshow(tuner_window, mask)
        
        key = cv.waitKey(15)
        if key == 27:
            break

    cv.destroyAllWindows()
