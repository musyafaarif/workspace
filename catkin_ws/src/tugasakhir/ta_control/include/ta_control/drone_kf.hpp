#include <ros/time.h>
#include <math.h>

#include "ta_control/kf.hpp"
// #include "ta_control/drone_vis.hpp"

#pragma once

class DroneKF : public KalmanFilter {
public:
    DroneKF() {
        m = 4; n = 4;
        F = MATRIX(m, n);
        // F << 1, 1, 0, 0,
        //      0, 1, 0, 0,
        //      0, 0, 1, 1,
        //      0, 0, 0, 1;
        F << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
        H = MATRIX(m, n);
        H << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
        Q = MATRIX(m, n);
        Q << 1e-2, 0, 0, 0,
             0, 1e-4, 0, 0,
             0, 0, 1e-2, 0,
             0, 0, 0, 1e-4;
        R = MATRIX(m, n);
        R << 1e-4, 0, 0, 0,
             0, 1e-6, 0, 0,
             0, 0, 1e-4, 0,
             0, 0, 0, 1e-6;
        P0 = MATRIX(m, n);
        P0 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        
        initialized = false;
        I = MATRIX(n, n); I.setIdentity();
        x_hat = VECTOR(n);
    }

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double rx, double ry) {
        VECTOR x0(m);
        x0 << rx, ry, 0, 0;
        KalmanFilter::init(x0);
        t = ros::Time::now();
    }

    /**
    * Predict the estimated state based on dt.
    */
    void predict() {
        double dt = (ros::Time::now() - t).toSec();

        F << 1, dt, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, dt,
             0, 0, 0, 1;
        KalmanFilter::predict();

        t = ros::Time::now();
    }

    /**
    * Update the estimated state based on measured values.
    */
//     void update(double rx, double ry, double px, double py, double theta, double phi, double h) {
//         VECTOR z(m);
//         z << rx, ry,
//              tan(px * Kx + phi) * h,
//              tan(py * Ky + theta) * h;
//         KalmanFilter::update(z);
//     }

    void update(double px, double vx, double py, double vy) {
          H = MATRIX(m, m);
          H << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

          R = MATRIX(m, n);
          R << 1e-1, 0, 0, 0,
               0, 1e-6, 0, 0,
               0, 0, 1e-1, 0,
               0, 0, 0, 1e-6;
        
          VECTOR z(4);
          z << px,
               vx,
               py,
               vy;
          // z << px,
          //      0,
          //      py,
          //      0;
          KalmanFilter::update(z);
    }

    void update(double px, double py) {
        H = MATRIX(2, 4);
        H << 1, 0, 0, 0,
             0, 0, 1, 0;

        R = MATRIX(2, 2);
        R << 1e-2, 0,
             0, 1e-2;
        
        VECTOR z(2);
        z << px,
             py;
        KalmanFilter::update(z);
    }

private:
    ros::Time t;
};

namespace drone {
     static DroneKF kf;
}