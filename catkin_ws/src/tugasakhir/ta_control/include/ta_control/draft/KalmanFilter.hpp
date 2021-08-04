#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#pragma once

class KalmanFilter {

public:

    /**
     * Create a Kalman filter with the specified matrices.
     *   F - System dynamics matrix
     *   B - System dynamics matrix
     *   H - Output matrix
     *   Q - Process noise covariance
     *   R - Measurement noise covariance
     *   P - Estimate error covariance
     */
    KalmanFilter(
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    ) : F(F), B(B), H(H), Q(Q), R(R), P0(P),
        m(H.rows()), n(F.rows()), initialized(false),
        I(n, n), x_hat(n), x_hat_new(n)
    {
        I.setIdentity()
    };

    /**
     * Initialize the filter with a guess for initial states.
     */
    void init(const Eigen::VectorXd& x0)
    {
        x_hat = x0;
        P = P0;
        initialized = true;
    };

    /**
     * Update the estimated state based on measured values. The
     * time step is assumed to remain constant.
     */
    void measure(const Eigen::VectorXd& y)
    {
        K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
        x_hat_new += K * (y - H*x_hat_new);
        P = (I - K*H)*P;
        x_hat = x_hat_new;
    };

    /**
     * Update the estimated state based on measured values,
     * using the given time step and dynamics matrix.
     */
    void predict()
    {

    };

    /**
     * Return the current state and time.
     */
    Eigen::VectorXd state() {
        return x_hat;
    };
    double time() { return t; };

private:

    // Matrices for computation
    Eigen::MatrixXd F, B, H, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time
    ros::Time t0, t;

    // Is the filter initialized?
    bool initialized;

    bool is_meas, is_pred;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;

protected:
    
};