#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include "ta_control/drone_kf.hpp"
#include "ta_control/math.hpp"
#include "ta_control/pid.hpp"
#include "ta_control/boat_com.hpp"
#include "ta_control/drone_com.hpp"
#include "ta_control/drone_control.hpp"
#include "ta_control/drone_vis.hpp"
#include "ta_control/scurve.hpp"
#include "ta_control/datalog.hpp"

#define HOVER_MISSION 101
#define CAM_MISSION 102

#define WEIGHT 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_follow");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Boat Communication
    boat::setupCom(nh);

    // Drone Communication
    drone::setupCom(nh);

    // Vision
    vision::init(nh);

    // Datalog
    datalog::init(nh);

    // wait for FCU connection
    ROS_INFO("Wait for Connection..");
    while (ros::ok()){
        if (drone::checkConnection() &&
            boat::checkConnection()
            // TODO: Check PID Status
        ) break;
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    drone::init(nh);

    hav::north_east distance;
    distance = hav::ne_distance(drone::nav.pos_global, boat::nav.pos_global);

    drone::pid->init(
        -distance.north,                    // X
        -distance.east,                     // Y
        0.0f,                               // Z
        drone::nav.heading_global.data,     // YAW
        0.0f,                               // Setpoint X
        0.0f,                               // Setpoint Y
        0.0f,                               // Setpoint Z
        boat::nav.heading_global.data       // Setpoint YAW
    );

    drone::kf.init(-distance.east, -distance.north);

    //send a few setpoints before starting
    ROS_INFO("Send a few setpoints..");
    for(int i = 20; ros::ok() && i > 0; --i){
        drone::setpoint_pub.publish(drone::setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    // DEBUG
    ros::Time log_time = ros::Time(0);
    hav::north_east centroid;

    // KALMAN FILTER
    geometry_msgs::PointStamped centroid_kf;

    // S-CURVE GUIDANCE
    double Xpeak[4] = {
        drone::target_takeoff_alt,  // Position
        1.0,                        // Velocity
        0.5,                        // Acceleration
        0.25                        // Jerk
    };
    double T[4] = {0, 0, 0, 0};
    ros::Time t0;
    ta_scurve::computePeriods(Xpeak, T);

    // ARMING
    drone::armGuided(rate);
    drone::pid->enable();

    // OFFSET
    distance = hav::ne_distance(
        drone::nav.pos_global.latitude, drone::nav.pos_global.longitude,
        boat::nav.pos_global.latitude, boat::nav.pos_global.longitude
    );
    drone::offset.north = distance.north;
    drone::offset.east = distance.east;
    ROS_INFO("OFFSET");
    ROS_INFO("N: %f; E: %f", drone::offset.north, drone::offset.east);

    // MISSION
    uint8_t mission                 = 1;
    double_t r                      = 0;
    drone::target.latitude          = -7.22975;
    drone::target.longitude         = 112.82999;
    drone::target.status.status     = drone::target.status.STATUS_NO_FIX;
    ros::Time centering_time        = ros::Time(0);

    ROS_INFO("Takeoff Altitude: %0.1f m", drone::target_takeoff_alt);
    while(ros::ok() && drone::state.mode == "OFFBOARD" && drone::state.armed){
        if (mission == 1) {
            if (drone::altitude() - drone::target_takeoff_alt >= -1e-1) {
                ROS_INFO("Altitude Reached!");

                // Re-init KF
                distance = hav::ne_distance(
                    drone::nav.pos_global.latitude, drone::nav.pos_global.longitude,
                    drone::target.latitude, drone::target.longitude
                );
                drone::kf.init(distance.north, distance.east);
                drone::kf.update(
                    distance.north - drone::offset.north,
                    0.0 - drone::nav.vel_global.twist.linear.x,
                    distance.east - drone::offset.east,
                    0.0 - drone::nav.vel_global.twist.linear.y
                );
                drone::pid->update_yaw(drone::nav.heading_global.data, 360.0 - ta_math::heading(distance.north, distance.east) + 90.0);

                mission++;
            }

            drone::updatePID();
        } else if (mission == 2) {
            drone::kf.predict();
            distance = hav::ne_distance(
                drone::nav.pos_global.latitude, drone::nav.pos_global.longitude,
                drone::target.latitude, drone::target.longitude
            );
            drone::kf.update(
                distance.north - drone::offset.north,
                0.0 - drone::nav.vel_global.twist.linear.x,
                distance.east - drone::offset.east,
                0.0 - drone::nav.vel_global.twist.linear.y
            );
            distance.north = drone::kf.state()[0]; distance.east = drone::kf.state()[2];

            r = abs(hav::distance(distance));
            if (r <= 1.0) {
                if (centering_time == ros::Time(0)) {
                    centering_time = ros::Time::now();
                } else if (ros::Time::now() - centering_time >= ros::Duration(2.0)) {
                    ROS_INFO("Target Reached!");

                    drone::target = boat::nav.pos_global;
                    // Re-init KF
                    distance = hav::ne_distance(
                        drone::nav.pos_global.latitude, drone::nav.pos_global.longitude,
                        drone::target.latitude, drone::target.longitude
                    );
                    drone::kf.init(distance.north, distance.east);
                    drone::kf.update(
                        distance.north - drone::offset.north,
                        0.0 - drone::nav.vel_global.twist.linear.x,
                        distance.east - drone::offset.east,
                        0.0 - drone::nav.vel_global.twist.linear.y
                    );

                    // Init centroid
                    centroid_kf.header.seq = vision::centroid.header.seq;

                    // Reset PID
                    drone::pid->reset();

                    drone::pid->update_yaw(drone::nav.heading_global.data, 360.0 - ta_math::heading(distance.north, distance.east) + 90.0);

                    mission++;
                }
            } else {
                centering_time = ros::Time(0);
            }

            drone::updatePID(distance);

            // Set yaw
            drone::pid->update_yaw(drone::nav.heading_global.data);

        } else if (mission == 3 || mission == 4) {
            drone::target = boat::nav.pos_global;

            drone::kf.predict();
            distance = hav::ne_distance(
                drone::nav.pos_global.latitude, drone::nav.pos_global.longitude,
                drone::target.latitude, drone::target.longitude
            );
            if (vision::centroid.header.seq != centroid_kf.header.seq) {
                centroid_kf = vision::centroid;
                centroid_kf.point.z = drone::altitude() + 0.186;
                centroid = vision::cam2pos(centroid_kf.point, drone::nav.att_imu.orientation, drone::nav.heading_global.data);
                // drone::kf.update(-distance.north, -distance.east);

                // WEIGHTING
                if (centroid.north <= 0.2)
                    distance.north = (1 - WEIGHT) * distance.north + (WEIGHT) * -centroid.north;
                if (centroid.east <= 0.2)
                    distance.east = (1 - WEIGHT) * distance.east + (WEIGHT) * -centroid.east;
            }


            drone::kf.update(
                distance.north - drone::offset.north,
                boat::nav.vel_global.twist.linear.x - drone::nav.vel_global.twist.linear.x,
                distance.east - drone::offset.east,
                boat::nav.vel_global.twist.linear.y - drone::nav.vel_global.twist.linear.y
            );
            distance.north = drone::kf.state()[0]; distance.east = drone::kf.state()[2];

            r = abs(hav::distance(distance));
            if (mission == 3 && r <= 5e-1) {
                if (centering_time == ros::Time(0)) {
                    centering_time = ros::Time::now();
                } else if (ros::Time::now() - centering_time >= ros::Duration(2.0)) {
                    ROS_INFO("Boat Reached!");
                    t0 = ros::Time::now();

                    // Reset PID
                    drone::pid->reset();
                    drone::pid->update_yaw(drone::nav.heading_global.data, boat::nav.heading_global.data - 5.0);

                    mission++;
                }
            } else if (mission == 4 && drone::altitude() <= 1e-1) {
                // ROS_INFO("Landed!");
                mission++;
                break;
            } else {
                centering_time = ros::Time(0);
            }

            if (mission == 3) {
                drone::updatePID(distance);
            }
            else {
                static double scurve_sp = 0;
                scurve_sp = ta_scurve::getSetpoint(Xpeak, T, (ros::Time::now() - t0).toSec());
                drone::updatePID(
                    distance,
                    drone::target_takeoff_alt - 
                    scurve_sp
                );


                datalog::scurve_log(scurve_sp);
            }
            if (r >= 4.0) {
                drone::pid->update_yaw(drone::nav.heading_global.data);
            }
            else {
                drone::pid->update_yaw(drone::nav.heading_global.data, boat::nav.heading_global.data - 5.0);
            }

        } else if (mission == HOVER_MISSION) {
            drone::target = boat::nav.pos_global;

            drone::kf.predict();

            // if (vision::centroid.header.seq != centroid_kf.header.seq) {
            //     centroid_kf = vision::centroid;
            //     centroid_kf.point.z = drone::altitude() + 0.186;
            //     distance = vision::cam2pos(centroid_kf.point, drone::nav.att_imu.orientation, drone::nav.heading_global.data);
            //     centroid = distance;
                
            //     kf.update(-distance.north, distance.east);
            //     // bugg = distance;
            // }

            distance.north = drone::kf.state()[0];
            distance.east = drone::kf.state()[2];

            drone::updatePID(distance, drone::target_takeoff_alt);
            drone::pid->update_yaw(drone::nav.heading_global.data, boat::nav.heading_global.data - 5.0);

            datalog::pid_log();
            datalog::kf_log(mission);
        } else if (mission == CAM_MISSION) {
            if (vision::centroid.header.seq != centroid_kf.header.seq) {
                centroid_kf = vision::centroid;
                centroid_kf.point.z = drone::altitude() + 0.186;
                distance = vision::cam2pos(centroid_kf.point, drone::nav.att_imu.orientation, drone::nav.heading_global.data);
                centroid = distance;

                datalog::cam::pos.point.x = distance.east;
                datalog::cam::pos.point.y = distance.north;

                datalog::cam::publish();
            }
            drone::updatePID(distance, 0);
        }

        drone::pid->publish();
        drone::setpoint_pub.publish(drone::setpoint);

        datalog::pid_log();
        if (ros::Time::now() - vision::centroid.header.stamp >= ros::Duration(0.5) || mission == 2)
            datalog::kf_log(mission);
        else
            datalog::kf_log(mission, centroid);

        // DEBUG
        if (ros::Time::now() - log_time > ros::Duration(2.0f))
        {
            // ROS_INFO("Distance: %0.3f m, Altitude: %0.3f m", r, drone::altitude());

            // debug = vision::centroid;
            // debug.point.z = drone::altitude() + 0.186;
            // distance = vision::cam2pos(debug.point, drone::nav.att_imu.orientation, drone::nav.heading_global.data);
            // ROS_INFO("N: %f\tE: %f", distance.north, distance.east);
            // ROS_INFO("X: %f\tY: %f", vision::centroid.point.x, vision::centroid.point.y);
            // ROS_INFO("DELAY: %f", (drone::nav.att_imu.header.stamp - vision::centroid.header.stamp).toSec());
            // ROS_INFO("N: %f, E: %f", bugg.north, bugg.east);

            // ROS_INFO("DRONE: %0.3f; BOAT: %0.3f",drone::setpoint.yaw_rate, boat::nav.heading_global.data);

            // state_kf = kf.state();
            // ROS_INFO("PX: %0.3f\tPY: %0.3f", state_kf(0), state_kf(2));
            // ROS_INFO("VX: %0.3f\tVY: %0.3f", kf_state(1), kf_state(3));

            // ROS_INFO("SP: %0.3f", ta_scurve::getSetpoint(Xpeak, T, (ros::Time::now() - t0).toSec()));
            log_time = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Landed!");
    datalog::flight_log();
    datalog::close();

    return 0;
}
