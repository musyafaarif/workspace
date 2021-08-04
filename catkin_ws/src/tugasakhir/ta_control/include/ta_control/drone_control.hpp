#include <ros/ros.h>
#include <string>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "ta_control/drone_com.hpp"
#include "ta_control/boat_com.hpp"
#include "ta_control/haversine.hpp"

#pragma once

namespace drone {
    static PID*                     pid;
    static double                   takeoff_alt;
    static double                   target_takeoff_alt;
    static sensor_msgs::NavSatFix   takeoff_pos;

    static mavros_msgs::PositionTarget  setpoint;
    static mavros_msgs::SetMode         set_mode;
    static mavros_msgs::CommandBool     cmd;
    static ros::Time                    last_request(0);

    void init(ros::NodeHandle& nh) {
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX
            | mavros_msgs::PositionTarget::IGNORE_AFY
            | mavros_msgs::PositionTarget::IGNORE_AFZ
            // | mavros_msgs::PositionTarget::IGNORE_VX
            // | mavros_msgs::PositionTarget::IGNORE_VY
            // | mavros_msgs::PositionTarget::IGNORE_VZ
            | mavros_msgs::PositionTarget::IGNORE_PX
            | mavros_msgs::PositionTarget::IGNORE_PY
            | mavros_msgs::PositionTarget::IGNORE_PZ
            | mavros_msgs::PositionTarget::IGNORE_YAW
            // | mavros_msgs::PositionTarget::IGNORE_YAW_RATE
        ; // END

        setpoint.velocity.x = 0.0f;
        setpoint.velocity.y = 0.0f;
        setpoint.velocity.z = 0.0f;
        setpoint.yaw_rate = 0.0;

        target_takeoff_alt = 2.0f;

        pid = new PID(&nh, &setpoint);
        pid->publish();
        // pid->update_z(nav.pos_global.altitude, target_takeoff_alt);
    }

    void armGuided(ros::Rate& rate) {
        set_mode.request.custom_mode = "OFFBOARD";
        cmd.request.value = true;

        while (ros::ok()) {
            if( drone::state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(set_mode) &&
                    set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !drone::state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(cmd) &&
                        cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            if (drone::state.armed) {
                takeoff_alt = nav.pos_global.altitude;
                takeoff_pos = nav.pos_global;
                return;
            }

            setpoint_pub.publish(setpoint);

            ros::spinOnce();
            rate.sleep();
        }

        return;
    }

    double altitude() {
        return nav.pos_global.altitude - takeoff_alt;
    }

    void updatePID() {
        pid->update_x(0);
        pid->update_y(0);
        pid->update_z(altitude(), target_takeoff_alt);
    }

    void updatePID(hav::north_east distance) {
        pid->update_x(-distance.north);
        pid->update_y(-distance.east);
        pid->update_z(altitude());
    }

    void updatePID(hav::north_east distance, double alt_setpoint) {
        pid->update_x(-distance.north);
        pid->update_y(-distance.east);
        pid->update_z(altitude(), alt_setpoint);
    }
}