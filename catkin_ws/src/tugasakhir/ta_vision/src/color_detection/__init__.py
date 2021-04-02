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

    def read(self):
        return self._mask

    def update(self, frame):
        frame_hsv = convert2hsv(frame)
        mask = create_mask(None, frame_hsv, self.lo, self.hi)
        self._mask = cv.medianBlur(mask, 3)

	    # calculate moments of binary image
        M = cv.moments(self._mask)

        self.has_centroid = M["m00"] != 0
        if self.has_centroid:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            self.centroid = (cX, cY)
        else:
            self.centroid = None

        return self._mask
