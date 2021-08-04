#!/usr/bin/env python
import ta_vision

from vision.camera import Camera
from color_detection import ColorDetection

import cv2 as cv
import rospy
import time

from geometry_msgs.msg import PointStamped

lower_threshold = (0, 140, 132)
upper_threshold = (10, 255, 255)
# lower_threshold = (160, 190, 220)
# upper_threshold = (180, 230, 255)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(15) # 15 FPS

        cam_pub = rospy.Publisher("camera/data", PointStamped, queue_size=10)

        cap = Camera(port=5600)
        cd = ColorDetection(lower_threshold, upper_threshold)

        rospy.loginfo("Wait for camera capture..")
        frame = cap.capture()
        while frame is None and not rospy.is_shutdown():
            rate.sleep()
            frame = cap.capture()
        rospy.loginfo("Frame captured!")
        
        fps = 15.0
        t = time.time()

        while not rospy.is_shutdown():
            frame = cap.capture()
            t_cap = rospy.Time.now()
            mask = cd.update(frame)

            if cd.centroid:
                (cX, cY) = cd.centroid
                centroid = PointStamped()
                centroid.point.x = cX - 160.0
                centroid.point.y = cY - 120.0
                centroid.point.y = -centroid.point.y
                centroid.header.stamp = t_cap
                cam_pub.publish(centroid)

            if cd.has_centroid:
                cv.circle(frame, cd.centroid, 5, 127, -1)
            cv.putText(frame, "fps: %.1f" % fps, (240, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, 127, 2)
            # if cd.has_centroid:
            #     cv.circle(mask, cd.centroid, 5, 127, -1)
            # cv.putText(mask, "fps: %.1f" % fps, (240, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, 127, 2)

            cv.imshow("Frame", frame)
            # cv.imshow("Frame", mask)
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
