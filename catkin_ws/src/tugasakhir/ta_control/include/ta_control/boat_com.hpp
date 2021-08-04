#include <ros/ros.h>

#include "ta_control/Navigation.h"

#pragma once

namespace boat {
    static ta_control::Navigation nav;

    void pos_global_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        nav.pos_global = *msg;
    }

    void vel_global_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        nav.vel_global = *msg;
    }

    void heading_cb(const std_msgs::Float64::ConstPtr& msg) {
        nav.heading_global = *msg;
    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
        nav.att_imu = *msg;
    }

    static ros::Subscriber pos_global_sub;
    static ros::Subscriber vel_global_sub;
    static ros::Subscriber heading_sub;
    static ros::Subscriber imu_sub;
    void setupCom(ros::NodeHandle& nh) {
        // pos_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
        //     ("boat_mavros/global_position/raw/fix", 10, pos_global_cb);
        // vel_global_sub = nh.subscribe<geometry_msgs::TwistStamped>
        //     ("boat_mavros/global_position/raw/gp_vel", 10, vel_global_cb);
        pos_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("boat_mavros/global_position/global", 10, pos_global_cb);
        vel_global_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("boat_mavros/local_position/velocity_local", 10, vel_global_cb);
        heading_sub = nh.subscribe<std_msgs::Float64>
            ("boat_mavros/global_position/compass_hdg", 10, heading_cb);
        imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("boat_mavros/imu/data", 10, imu_cb);
    }

    bool checkConnection()
    {
        return
            // nav.pos_global.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
            nav.pos_global.latitude != 0 && nav.pos_global.longitude != 0;
    }
}
