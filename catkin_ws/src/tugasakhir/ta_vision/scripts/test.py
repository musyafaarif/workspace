#!/usr/bin/env python2

import rospy as ros
import cv2 as cv

if __name__ == "__main__":
    try:
        ros.init_node("vision")
        rate = ros.Rate(15)         # 15 FPS
        cap = cv.VideoCapture(0)

        while not ros.is_shutdown():
            _, frame = cap.read()
            if frame is None:
                break

            size = (320, 240)
            frame = cv.resize(frame, size)

            cv.imshow("Frame", frame)
            key = cv.waitKey(66)
            if key == 27:
                break

            rate.sleep()

    except ros.ROSInterruptException:
        pass