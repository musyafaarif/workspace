#!/usr/bin/env python
import ta_vision

# import color_detection
from vision.camera import Camera
import color_detection
import cv2 as cv
import rospy
import time

lower_threshold = (0, 0, 0)
upper_threshold = (100, 100, 100)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(15) # 15 FPS

        cap = Camera(src=0)
        cd = color_detection.ColorDetection(lower_threshold, upper_threshold)

        t = time.time()

        while not rospy.is_shutdown():
            frame = cap.capture()

            mask = cd.update(frame)

            cv.imshow("Frame", mask)
            key = cv.waitKey(15)
            if key == 27:
                break

            print("FPS: = ", 1/(time.time() - t))
            t = time.time()
            rate.sleep()


    except rospy.ROSInterruptException:
        pass

    if cap is not None:
        cap.close()
    cv.destroyAllWindows()