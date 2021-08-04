#include <eigen3/Eigen/Dense>

#pragma once

using namespace Eigen;

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(
        const MatrixXd& F,
        const MatrixXd& H,
        const MatrixXd& Q,
        const MatrixXd& R
    ) : F(F), H(H), Q(Q), R(R)
    {

    }

    ExtendedKalmanFilter()
    {

    }
    void init(const MatrixXd& x0, const MatrixXd& P0)
    {
        x_hat = x0;
        P = P0;
    }

    void kfPredict()
    {
        x_hat = F * x_hat;
        P = F * P * F.transpose() + Q;
    }

    void kfUpdate(const VectorXd &z)
    {
        VectorXd y = z - H * x_hat;
        MatrixXd S = H * P * H.transpose() + R;
        
        K = P * H.transpose() * S.inverse();
        x_hat = x_hat + K * y;

        size_t x_size = x_hat.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P = (I - K * H) * P;
    }


    void ekfPredict(
        VectorXd (*f)(const VectorXd& x, const VectorXd& u, double_t dt),
        const VectorXd& u,
        const MatrixXd& Fj,
        const MatrixXd& Q,
        double dt
    )
    {
        // Set f function
        x_hat << f(x_hat, u, dt);
        P = Fj * P * Fj.transpose() + Q;
    }

    void ekfUpdate(
        VectorXd (*h)(const VectorXd& x),
        const VectorXd& z,
        const MatrixXd& Hj,
        const MatrixXd& R
    )
    {
        VectorXd y = z - h(x_hat);
        MatrixXd S = Hj * P * Hj.transpose() + R;
        
        K = P * Hj.transpose() * S.inverse();
        x_hat = x_hat + K * y;

        size_t x_size = x_hat.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P = (I - K * Hj) * P;
    }

    VectorXd x(){ return x_hat; }

private:
    MatrixXd F, H, Q, R, P, K;

    VectorXd x_hat;
};
