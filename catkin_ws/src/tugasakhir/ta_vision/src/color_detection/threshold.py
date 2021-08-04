from color_detection.histogram import hsv

import cv2 as cv
import numpy as np


class Threshold(hsv):
    def __init__(self):
        hsv.__init__(self)

    def assign(self, img):
        self.img = img
        self.img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        self._hsv__assign(self.img_hsv)

    def read_mask(self):
        return cv.inRange(self.img_hsv,
                          np.array([self.hue.range[0],
                                    self.sat.range[0],
                                    self.val.range[0]]),
                          np.array([self.hue.range[1],
                                    self.sat.range[1],
                                    self.val.range[1]]))

    def read(self):
        return cv.bitwise_or(self.img, self.img, mask=self.read_mask())

    def update(self, filter):
        if self.__filter(filter):
            self.sat.value, self.sat.range, _ = \
                self.find_sat()
            self.val.value, self.val.range, _ = \
                self.find_val()
        else:
            self.sat.value = 0
            self.sat.range = np.zeros(2)
            self.val.value = 0
            self.val.range = np.zeros(2)

    def __filter(self, filter):
        if self.hue.color != "red":
            # Filter hue from histogram
            self.hue.value, self.hue.range, retval = \
                self.find_hue(filter)

            if retval == -1:
                # Masking
                mask = cv.inRange(
                    self.img_hsv,
                    np.array([self.hue.range[0],
                              self.sat.find_range[0],
                              self.val.find_range[0]]),
                    np.array([self.hue.range[1],
                              self.sat.find_range[1],
                              self.val.find_range[1]]))
                self.img_hsv_filtered = cv.bitwise_and(
                    self.img_hsv, self.img_hsv, mask=mask
                )

                self._hsv__assign(self.img_hsv_filtered)

        return retval == -1
