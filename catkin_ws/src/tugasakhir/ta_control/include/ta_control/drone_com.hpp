#include <ros/ros.h>

#include "ta_control/Navigation.h"
#include "ta_control/haversine.hpp"

#pragma once

namespace drone {
    static hav::north_east          offset;
    static sensor_msgs::NavSatFix   target;
    static mavros_msgs::State       state;
    static ta_control::Navigation   nav;

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        state = *msg;
    }

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

    static ros::Publisher       setpoint_pub;
    static ros::Subscriber      pos_global_sub;
    static ros::Subscriber      vel_global_sub;
    static ros::Subscriber      heading_sub;
    static ros::Subscriber      imu_sub;
    static ros::Subscriber      state_sub;
    static ros::ServiceClient   arming_client;
    static ros::ServiceClient   set_mode_client;
    void setupCom(ros::NodeHandle& nh) {
        setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("drone_mavros/setpoint_raw/local", 10);
        // pos_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
        //     ("drone_mavros/global_position/raw/fix", 10, pos_global_cb);
        // vel_global_sub = nh.subscribe<geometry_msgs::TwistStamped>
        //     ("drone_mavros/global_position/raw/gp_vel", 10, vel_global_cb);
        pos_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("drone_mavros/global_position/global", 10, pos_global_cb);
        vel_global_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("drone_mavros/local_position/velocity_local", 10, vel_global_cb);
        heading_sub = nh.subscribe<std_msgs::Float64>
            ("drone_mavros/global_position/compass_hdg", 10, heading_cb);
        imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("drone_mavros/imu/data", 10, imu_cb);
        state_sub = nh.subscribe<mavros_msgs::State>
            ("drone_mavros/state", 10, state_cb);

        // Services
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("drone_mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("drone_mavros/set_mode");
    }

    bool checkConnection()
    {
        return state.connected
            && abs(nav.pos_global.latitude) >= 1e-2
            && abs(nav.pos_global.longitude) >= 1e-2;
    }
}
