#!/usr/bin/env python
import ta_vision

from vision.camera import Camera
from color_detection import ColorDetection

import cv2 as cv
import rospy
import time

from geometry_msgs.msg import Point

lower_threshold = (17, 166, 36)
upper_threshold = (23, 255, 51)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(15) # 15 FPS

        centroid_pub = rospy.Publisher("~/centroid", Point, tcp_nodelay=True, queue_size=1)

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

            cv.putText(mask, "fps: %.1f" % fps, (240, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, 127, 2)

            centroid = Point(x = 0, y = 0, z = cd.has_centroid)
            if cd.has_centroid:
                centroid.x = cd.centroid[0] - cap.res[0] / 2
                centroid.y = -(cd.centroid[1] - cap.res[1] / 2)
                cv.circle(mask, cd.centroid, 5, 127, -1)

            centroid_pub.publish(centroid)

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