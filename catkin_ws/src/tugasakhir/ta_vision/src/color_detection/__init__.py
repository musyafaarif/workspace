import cv2 as cv
import numpy as np

def convert2hsv(frame):
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    return frame_hsv

def create_mask(frame=None, frame_hsv=None, lower_threshold=(0, 0, 0), upper_threshold=(255, 255, 255)):
    if frame_hsv is None:
        frame_hsv = convert2hsv(frame)
    
    mask = cv.inRange(frame_hsv, np.array(lower_threshold), np.array(upper_threshold))
    return mask

class ColorDetection(object):
    def __init__(self, lower_threshold, upper_threshold):
        self.lo = np.array(lower_threshold)
        self.hi = np.array(upper_threshold)

    def update(self, frame):
        frame_hsv = convert2hsv(frame)
        mask = create_mask(None, frame_hsv, self.lo, self.hi)
        mask_clean = cv.medianBlur(mask, 11)
        return mask_clean
