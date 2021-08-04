#include <eigen3/Eigen/Dense>

#pragma once

using namespace Eigen;

VectorXd f(const VectorXd& x, const VectorXd& u, double_t dt)
{
    size_t x_size = x.size();
    VectorXd x_hat(x_size);
    x_hat <<
        x(0) + x(1) * dt,   // sx
        x(1),               // vx
        x(2) + x(3) * dt,   // sy
        x(3),               // vy
        x(4) + x(5) * dt,   // sz
        x(5),               // vz
        x(6) + x(7) * dt,   // theta
        x(7),               // delta theta
        x(8) + x(9) * dt,   // phi
        x(9);               // delta phi

    return x_hat;
}

MatrixXd Fj(const VectorXd& x, const VectorXd& u, double_t dt)
{
    size_t x_size = x.size();
    MatrixXd F(x_size, x_size);
    F <<
        1, dt, 0, 0, 0, 0, 0, 0, 0, 0,  // sx
        0,  1, 0, 0, 0, 0, 0, 0, 0, 0,  // vx
        0, 0, 1, dt, 0, 0, 0, 0, 0, 0,  // sy
        0, 0, 0,  1, 0, 0, 0, 0, 0, 0,  // vy
        0, 0, 0, 0, 1, dt, 0, 0, 0, 0,  // sz
        0, 0, 0, 0, 0,  1, 0, 0, 0, 0,  // vz
        0, 0, 0, 0, 0, 0, 1, dt, 0, 0,  // theta
        0, 0, 0, 0, 0, 0, 0,  1, 0, 0,  // delta theta
        0, 0, 0, 0, 0, 0, 0, 0, 1, dt,  // phi
        0, 0, 0, 0, 0, 0, 0, 0, 0,  1;  // delta phi

    return F;
}

MatrixXd Q(const VectorXd& x)
{
    MatrixXd Q(9, 9);
    Q <<
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    return Q * .01;
}

VectorXd h_gps(const VectorXd& x)
{
    VectorXd h_(x.size());
    h_ <<
        x(0),
        x(1),
        x(2),
        x(3),
        x(4),
        x(5),
        x(6),
        x(7),
        x(8),
        x(9);

    return h_;
}

MatrixXd Hj_gps(const VectorXd& x)
{
    MatrixXd H(9, 9);
    H <<
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    return H;
}

MatrixXd R_gps(const VectorXd& x)
{
    MatrixXd R(9, 9);
    R <<
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    return R * .01;
}

VectorXd h_cam(const VectorXd& x)
{
    const double Kx = 1129;
    const double Ky = 1185;

    double sx = x(0);
    double sy = x(2);
    double h = x(4);
    double theta = x(6);
    double phi = x(8);

    VectorXd h_(2);
    h_ <<
        Kx * (atan(sx / h) + theta),
        Ky * (atan(sy / h) + phi);

    return h_;
}

MatrixXd Hj_cam(const VectorXd& x)
{
    const double Kx = 1129;
    const double Ky = 1185;

    double sx = x(0);
    double sy = x(2);
    double h = x(4);

    MatrixXd H(2, 4);
    H <<
        Kx / (h * (1 + pow(sx / h, 2))), 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, Ky / h * ((1 + pow(sy / h, 2))), 0, 0, 0, 0, 0, 0, 0;

    return H;
}

MatrixXd R_cam(const VectorXd& x)
{
    MatrixXd R(2, 2);
    R <<
        1, 0,
        0, 1;

    return R * 16;
}
