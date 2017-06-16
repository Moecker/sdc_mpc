#ifndef MPC_AUXILIARY_H
#define MPC_AUXILIARY_H

#include <math.h>
#include <cppad/cppad.hpp>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

constexpr double Pi()
{
    return M_PI;
}

double Deg2Rad(double x);
double Rad2Deg(double x);

CppAD::AD<double> Polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

template <typename T>
void SimulateTimestep(T& px,
                      T& py,
                      T& phi,
                      T& velocity,
                      const T steering,
                      const T throttle,
                      const double dt,
                      const double Lf)
{
    px += velocity * cos(phi) * dt;
    py += velocity * sin(phi) * dt;
    phi += velocity / Lf * steering * dt;
    velocity += throttle * dt;
}

#endif  // MPC_AUXILIARY_H
