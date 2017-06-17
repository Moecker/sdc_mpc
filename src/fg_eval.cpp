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
    AD<double> speed = vars[3];

    for (int i = 0; i < MPC::N - 1; i++)
    {
        AD<double> steering = vars[MPC::kStateSize + i];
        AD<double> throttle = vars[MPC::kStateSize + (MPC::N - 1) + i];

        // Here we call the simulation step to produce the next state varables for each ith step.
        SimulateTimestep(px, py, phi, speed, steering, throttle, MPC::dt, MPC::Lf);

        const auto postion_cost = position_weight_ * CppAD::pow(Polyeval(coeffs, px) - py, 2);
        const auto velocity_cost = speed_weight_ * CppAD::pow(speed - v_reference_, 2);
        const auto steering_cost = steering_weight_ * CppAD::pow(steering, 2);
        const auto trottle_cost = throttle_weight_ * CppAD::pow(throttle, 2);

        // Consideration of the cost
        fg[0] += postion_cost + velocity_cost + steering_cost + trottle_cost;

        // Consideration of the angle
        fg[i + 1] = phi;
    }
}
