#include "fg_eval.h"
#include "auxiliary.h"
#include "mpc.h"

using CppAD::AD;

void FgEvaluator::operator()(ADvector& fg, const ADvector& vars)
{
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]
    fg[0] = 0;

    AD<double> px = vars[0];
    AD<double> py = vars[1];
    AD<double> phi = vars[2];
    AD<double> ve = vars[3];

    for (int i = 0; i < MPC::N - 1; i++)
    {
        AD<double> steering = vars[MPC::kStateSize + i];
        AD<double> throttle = vars[MPC::kStateSize + (MPC::N - 1) + i];

        SimulateTimestep(px, py, phi, ve, steering, throttle, MPC::dt, MPC::Lf);

        const auto postion_cost = alpha_ * CppAD::pow(Polyeval(coeffs, px) - py, 2);
        const auto velocity_cost = beta_ * CppAD::pow(ve - v_reference_, 2);
        const auto lambda_solution = lambda_ * CppAD::pow(steering, 2);
        const auto trottle_cost = nu_ * CppAD::pow(throttle, 2);

        fg[0] += postion_cost + velocity_cost + lambda_solution + trottle_cost;
        fg[i + 1] = phi;  // Total angle
    }
}
