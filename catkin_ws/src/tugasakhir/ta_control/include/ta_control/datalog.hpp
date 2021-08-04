#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <fstream>

#include "ta_control/drone_control.hpp"
#include "ta_control/drone_kf.hpp"

#pragma once

#define OCEAN   0
#define BOAT    1
#define DRONE   2

namespace datalog {
    namespace cam {
        static geometry_msgs::PointStamped  pos;
        static ros::Publisher               pub;

        void publish() {
            pub.publish(pos);
        }
    }

    namespace kf {
        static geometry_msgs::PointStamped  pos;
        static ros::Publisher               pub;

        void publish() {
            pub.publish(pos);
        }
    }

    static gazebo_msgs::ModelStates models;
    void models_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        models = *msg;
    }

    static geometry_msgs::Point offset;
    static std::fstream pid_csv;
    static std::fstream kf_csv;
    static std::fstream scurve_csv;
    static std::fstream flight_csv;
    static ros::Subscriber models_sub;
    void init(ros::NodeHandle nh) {
        cam::pub = nh.advertise<geometry_msgs::PointStamped>
            ("/datalog/cam", 10);
        kf::pub = nh.advertise<geometry_msgs::PointStamped>
            ("/datalog/kf", 10);
        
        models_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("/gazebo/model_states", 10, models_cb);
        
        pid_csv.open("/home/musyafa/Datalog/pid.csv", std::ios::out);
        pid_csv << "Time, Setpoint X, State X, Control X, Setpoint Y, State Y, Control Y, Setpoint Z, State Z, Control Z\n";

        gazebo_msgs::ModelStates msg = *(ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states"));
        offset.x = msg.pose[DRONE].position.x - msg.pose[BOAT].position.x;
        offset.y = msg.pose[DRONE].position.y - msg.pose[BOAT].position.y;
        offset.z = msg.pose[DRONE].position.z - msg.pose[BOAT].position.z;

        kf_csv.open("/home/musyafa/Datalog/kf.csv", std::ios::out);
        kf_csv << "Time, Kalman Position X, Kalman Velocity X, Kalman Position Y, Kalman Velocity Y, Real Position X, Real Velocity X, Real Position Y, Real Velocity Y, North From Cam, East From Cam\n";

        scurve_csv.open("/home/musyafa/Datalog/scurve.csv", std::ios::out);
        scurve_csv << "Time, Setpoint Altitude, Altitude, Velocity Z, Acceleration Z\n";

        flight_csv.open("/home/musyafa/Datalog/flight.csv", std::ios::app);
        // if (flight_csv.peek() == std::fstream::traits_type::eof()) {
        //     flight_csv << "Flight, Error Position X, Error Position Y";
        // }
    }

    void pid_log() {
        pid_csv << ros::Time::now() << ", "
                << drone::pid->setpoint_x.data << ", "
                << drone::pid->state_x.data << ", "
                << drone::pid->control_x.data << ", "
                << drone::pid->setpoint_y.data << ", "
                << drone::pid->state_y.data << ", "
                << drone::pid->control_y.data << ", "
                << drone::pid->setpoint_z.data << ", "
                << drone::pid->state_z.data << ", "
                << drone::pid->control_z.data << "\n";
    }

    void kf_log(uint8_t mission, hav::north_east ne) {
        if (mission != 2) {
            kf_csv  << ros::Time::now() << ", "
                    << drone::kf.state()[0] << ", "
                    << drone::kf.state()[1] << ", "
                    << drone::kf.state()[2] << ", "
                    << drone::kf.state()[3] << ", "
                    << models.pose[BOAT].position.x -models.pose[DRONE].position.x - offset.x << ", "
                    << models.twist[BOAT].linear.x - models.twist[DRONE].linear.x << ", "
                    << models.pose[BOAT].position.y -models.pose[DRONE].position.y - offset.y << ", "
                    << models.twist[BOAT].linear.y - models.twist[DRONE].linear.y << ", "
                    << -ne.north << ", "
                    << -ne.east << "\n";
        } else {
            static hav::north_east distance = hav::ne_distance(drone::takeoff_pos, drone::target);
            kf_csv  << ros::Time::now() << ", "
                    << drone::kf.state()[0] << ", "
                    << drone::kf.state()[1] << ", "
                    << drone::kf.state()[2] << ", "
                    << drone::kf.state()[3] << ", "
                    << distance.north -models.pose[DRONE].position.x - offset.x << ", "
                    << - models.twist[DRONE].linear.x << ", "
                    << distance.east -models.pose[DRONE].position.y - offset.y << ", "
                    << - models.twist[DRONE].linear.y << ", "
                    << 0 << ", "
                    << 0 << "\n";
        }
    }

    void kf_log(uint8_t mission) {
        hav::north_east ne;
        ne.north = 0; ne.east = 0;
        kf_log(mission, ne);
    }

    void scurve_log(double sp) {
        scurve_csv  << ros::Time::now() << ", "
                    << drone::target_takeoff_alt - sp << ", "
                    << drone::altitude() << ", "
                    << drone::nav.vel_global.twist.linear.z << ", "
                    << drone::nav.att_imu.linear_acceleration.z << "\n";
    }

    void flight_log() {
        EulerAngles angle = ta_math::q2eu(models.pose[BOAT].orientation);

        double x = models.pose[DRONE].position.x -models.pose[BOAT].position.x - offset.x;
        double y = models.pose[DRONE].position.y -models.pose[BOAT].position.y - offset.y;
        double landing_front = cos(angle.yaw) * x + sin(angle.yaw) * y;
        double landing_right = sin(angle.yaw) * x - cos(angle.yaw) * y;
        flight_csv  << ", "
                    << landing_right << ", "
                    << landing_front << "\n";
    }

    void close() {
        pid_csv.close();
        kf_csv.close();
        scurve_csv.close();
        flight_csv.close();
    }
}