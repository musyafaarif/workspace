#!/usr/bin/env python
import ta_vision

from vision.camera import Camera
from color_detection import ColorDetection

import cv2 as cv
import rospy
import time

lower_threshold = (17, 166, 36)
upper_threshold = (23, 255, 51)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(15) # 15 FPS

        cap = Camera(port=5600)
        cd = ColorDetection(lower_threshold, upper_threshold)

        print("Wait for camera capture..")
        frame = cap.capture()
        while frame is None and not rospy.is_shutdown():
            rate.sleep()
            frame = cap.capture()
        print("Frame captured!")
        
        fps = 15.0
        t = time.time()

        while not rospy.is_shutdown():
            frame = cap.capture()
            mask = cd.update(frame)

            if cd.has_centroid:
                cv.circle(mask, cd.centroid, 5, 127, -1)
            cv.putText(mask, "fps: %.1f" % fps, (240, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, 127, 2)

            cv.imshow("Frame", mask)
            key = cv.waitKey(15)
            if key == 27:
                break

            fps =  0.9 * fps + 0.1 * 1 / (time.time() - t)
            t = time.time()
            rate.sleep()


    except rospy.ROSInterruptException:
        pass

    if cap is not None:
        cap.close()
    cv.destroyAllWindows()