#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

OCEAN = 0
BOAT = 1
DRONE = 2

models = ModelStates()
def models_cb(msg):
    global models
    models = msg

if __name__ == "__main__":
    try:
        rospy.init_node('logging')
        rospy.Subscriber('/gazebo/model_states', ModelStates, models_cb)

        rate = rospy.Rate(0.5)
        t = rospy.Time(0)

        rospy.wait_for_message('/gazebo/model_states', ModelStates)
        offset_x = models.pose[DRONE].position.x - models.pose[BOAT].position.x
        offset_y = models.pose[DRONE].position.y - models.pose[BOAT].position.y
        offset_z = models.pose[DRONE].position.z - models.pose[BOAT].position.z

        while not rospy.is_shutdown():
            drone_x = models.pose[DRONE].position.x - models.pose[BOAT].position.x - offset_x
            drone_y = models.pose[DRONE].position.y - models.pose[BOAT].position.y - offset_y
            drone_z = models.pose[DRONE].position.z - models.pose[BOAT].position.z - offset_z

            if rospy.Time.now() - t >= rospy.Duration(2.0):
                rospy.loginfo("X: %0.3f, Y: %0.3f, Z: %0.3f", drone_x, drone_y, drone_z)
                # rospy.loginfo("X: %f, Y: %f, Z: %f", models.pose[BOAT].position.x, models.pose[BOAT].position.y, models.pose[BOAT].position.z)
                t = rospy.Time.now()

            rate.sleep()

        pass
    except rospy.ROSInterruptException:
        pass
