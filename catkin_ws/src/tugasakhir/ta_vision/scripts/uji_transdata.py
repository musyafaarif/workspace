#!/usr/bin/env python3
import ta_vision

from vision.camera import Camera
from color_detection import ColorDetection

import cv2 as cv
import rospy
import time
import math

from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import csv

lower_threshold = (160, 190, 220)
upper_threshold = (180, 230, 255)

FW = 320
FH = 240
FOVX = 62.2
FOVY = 48.8
KX = FOVX / FW / 180.0 * math.pi
KY = FOVY / FH / 180.0 * math.pi

# CSV
cam_csv = open('/home/musyafa/Datalog/cam.csv', 'w')
real_csv = open('/home/musyafa/Datalog/real.csv', 'w')

cam_writer = csv.writer(cam_csv)
real_writer = csv.writer(real_csv)

cam_writer.writerow(['Time', 'Cam Position X', 'Cam Position Y', 'Time', 'Real Position X', 'Real Position Y'])
real_writer.writerow(['Time', 'Real Position X', 'Real Position Y'])

waktu = rospy.Time(0)

pose = Pose()
z = 0
roll = 0
pitch = 0
yaw = 0
def models_cb(msg):
    global pose, z, roll, pitch, yaw, waktu
    pose = msg.pose[1]
    z = msg.pose[1].position.z
    orientation_list = [msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    waktu = rospy.Time.now()

def trans_data(x, y):
    global z, roll, pitch, yaw

    x_ = math.tan(KX * x - roll) * z
    y_ = math.tan(KY * y - pitch) * z

    # x_ = math.tan(KX * x + roll) * z
    # y_ = math.tan(KY * y + pitch - 1.57079632679) * z

    out_x = -math.sin(yaw) * x_ - math.cos(yaw) * y_
    out_y = math.cos(yaw) * x_ - math.sin(yaw) * y_

    return (out_x, out_y)

if __name__ == "__main__":
    try:
        rospy.init_node("color_detection")
        rate = rospy.Rate(15) # 15 FPS

        cam_pos_pub = rospy.Publisher('/datalog/cam', Point, queue_size=5)
        real_pos_pub = rospy.Publisher('/datalog/real', Point, queue_size=5)
        cam_pub = rospy.Publisher("camera/data", PointStamped, queue_size=10)

        rospy.Subscriber('/gazebo/model_states', ModelStates, models_cb)

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.wait_for_message('/gazebo/model_states', ModelState)
        model = ModelState()
        model.model_name='pi_cam'
        model.pose = pose
        model.pose.position.x = -0.5
        model.pose.position.y = -0.5
        set_state(model_state=model)

        cap = Camera(port=5600)
        cd = ColorDetection(lower_threshold, upper_threshold)

        rospy.loginfo("Wait for camera capture..")
        frame = cap.capture()
        while frame is None and not rospy.is_shutdown():
            rate.sleep()
            frame = cap.capture()
        rospy.loginfo("Frame captured!")

        fps = 30.0
        t = time.time()

        while not rospy.is_shutdown():
            frame = cap.capture()
            t_cap = rospy.Time.now()
            mask = cd.update(frame)

            if cd.centroid:
                (cX, cY) = cd.centroid
                centroid = PointStamped()
                centroid.point.x = cX - 160
                centroid.point.y = cY - 120
                centroid.point.y = -centroid.point.y
                centroid.header.stamp = t_cap
                cam_pub.publish(centroid)

                (X, Y) = trans_data(centroid.point.x, centroid.point.y)
                rospy.loginfo("ERRX: %f; ERRY: %f", X - pose.position.x, Y - pose.position.y)
                cam_pos = Point(x=X, y=Y, z=1)
                cam_pos_pub.publish(cam_pos)

                cam_writer.writerow([t_cap, cam_pos.x, cam_pos.y, waktu, pose.position.x, pose.position.y])
                real_writer.writerow([waktu, pose.position.x, pose.position.y])

            pose.position.x = pose.position.x + 0.001
            pose.position.y = pose.position.y + 0.001
            model.pose = pose
            set_state(model_state=model)

            real_pos_pub.publish(pose.position)

            if pose.position.x >= 0.5:
                break

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

    cam_csv.close()
    real_csv.close()
