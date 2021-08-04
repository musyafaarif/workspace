#include <ros/ros.h>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "ta_control/drone_kf.hpp"
#include "ta_control/drone_control.hpp"
#include "ta_control/math.hpp"
#include "ta_control/haversine.hpp"

#pragma once

#define Fw 320
#define Fh 240
#define fovx 62.2
#define fovy 48.8
#define Kx fovx / Fw
#define Ky fovy / Fh

namespace vision {
    geometry_msgs::Point cam2pos(geometry_msgs::Point P_in, geometry_msgs::Quaternion q);
    hav::north_east cam2pos(geometry_msgs::Point P_in, geometry_msgs::Quaternion q, double yaw);

    geometry_msgs::PointStamped centroid;
    void centroid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
        centroid = *msg;
    }

    geometry_msgs::Point cam2pos(geometry_msgs::Point P_in, geometry_msgs::Quaternion q) {
        // Transform quaternion to Euler
        EulerAngles angle = ta_math::q2eu(q);

        geometry_msgs::Point P_out;
        P_out.x = tan(ta_math::deg2rad(Kx * P_in.x) + angle.roll) * P_in.z;
        P_out.y = tan(ta_math::deg2rad(Ky * P_in.y) + angle.pitch) * P_in.z;
        P_out.z = P_in.z;

        return P_out;
    }

    hav::north_east cam2pos(geometry_msgs::Point P_in, geometry_msgs::Quaternion q, double yaw) {
        // Transform quaternion to Euler
        EulerAngles angle = ta_math::q2eu(q);

        geometry_msgs::Point P_out;
        P_out.x = tan(ta_math::deg2rad(Kx * P_in.x) - angle.roll) * P_in.z;
        P_out.y = tan(ta_math::deg2rad(Ky * P_in.y) - angle.pitch) * P_in.z;
        P_out.z = P_in.z;

        hav::north_east ne;
        yaw = ta_math::deg2rad(yaw);
        ne.north    = -1.0 * sin(yaw) * P_out.x     + -1.0 * cos(yaw) * P_out.y;
        ne.east     = cos(yaw) * P_out.x            + -1.0 * sin(yaw) * P_out.y;

        return ne;
    }

    ros::Subscriber centroid_sub;
    void init(ros::NodeHandle nh) {
        centroid_sub = nh.subscribe<geometry_msgs::PointStamped>(
            "/camera/data", 10, centroid_cb);
    }

    bool checkConnection() {
        return true;
    }
}
