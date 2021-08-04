#include "ta_control/ekf.hpp"

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include "ta_control/ekf_func.hpp"

#pragma once

#define SENSOR_GPS 0
#define SENSOR_CAM 1

class Fusion
{
public:
    Fusion(){}
    VectorXd x()
    {
        predict();
        return ekf.x();
    }

    void predict(){
        double dt = (ros::Time::now() - last_fusion).toSec();

        ekf.ekfPredict(
            &f,
            VectorXd(0),
            Fj(ekf.x(), VectorXd(0), dt),
            Q(ekf.x()),
            dt
        );

        last_fusion = ros::Time::now();
        return;
    }

    void update(const VectorXd& z, id_t sensor_type)
    {
        if (sensor_type == SENSOR_GPS)
        {
            ekf.ekfUpdate(&h_gps, z, Hj_gps(ekf.x()), R_gps(ekf.x()));
        }
        else if (sensor_type == SENSOR_CAM)
        {
            ekf.ekfUpdate(&h_cam, z, Hj_cam(ekf.x()), R_cam(ekf.x()));
        }

        last_fusion = ros::Time::now();
    }

private:
    ExtendedKalmanFilter ekf;
    ros::Time last_fusion;
};
