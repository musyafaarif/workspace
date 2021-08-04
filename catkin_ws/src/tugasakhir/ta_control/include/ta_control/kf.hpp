#include <mutex>
#include <eigen3/Eigen/Dense>

#pragma once

#define MATRIX Eigen::MatrixXd
#define VECTOR Eigen::VectorXd

class KalmanFilter {
public:
    KalmanFilter(
        const MATRIX& F,
        const MATRIX& H,
        const MATRIX& Q,
        const MATRIX& R,
        const MATRIX& P0)
    {
        this->F = F;
        this->H = H;
        this->Q = Q;
        this->R = R;
        this->P0 = P0;
        m = H.rows();
        n = F.rows();
        initialized = false;
        I = MATRIX(n, n); I.setIdentity();
        x_hat = VECTOR(n);
    }

    /**
    * Create a blank estimator.
    */
    KalmanFilter() {

    }

    /**
    * Initialize the filter with initial states as zero.
    */
    void init() {
        x_hat.setZero();
        P = P0;
        initialized = true;
    }

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(const VECTOR& x0) {
        init();
        x_hat = x0;
    }

    /**
    * Predict the estimated state based on new F matrix.
    */
    void predict() {
        if (!initialized)
            throw std::runtime_error("Filter is not initialized!");

        mtx.lock();
        x_hat = F * x_hat;
        P = F * P * F.transpose() + Q;
        mtx.unlock();
    }

    /**
    * Predict the estimated state based on new F matrix.
    */
    void predict(const MATRIX& F_new) {
        F = F_new;
        predict();
    }

    /**
    * Predict the estimated state based on new F matrix and new Q.
    */
    void predict(const MATRIX& F_new, const MATRIX& Q_new) {
        F = F_new;
        Q = Q_new;
        predict();
    }

    /**
    * Update the estimated state based on measured values.
    */
    void update(const VECTOR& z) {
        if(!initialized)
            throw std::runtime_error("Filter is not initialized!");
        
        mtx.lock();
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x_hat += K * (z - H * x_hat);
        P = (I - K * H) * P;
        mtx.unlock();
    }

    /**
    * Return the current state and time.
    */
    VECTOR state() { return x_hat; }

    bool is_initialized() { return initialized;}

protected:
    // Matrices for computation
    MATRIX F, H, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    MATRIX I;

    // Estimated states
    VECTOR x_hat;

private:
    // Lock
    std::mutex mtx;
};
