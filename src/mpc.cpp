#include <cppad/ipopt/solve.hpp>

#include "auxiliary.h"
#include "fg_eval.h"
#include "mpc.h"

using CppAD::AD;

MPC::MPC()
{
    historic_steering.resize(N - 1);
    historic_throttle.resize(N - 1);

    // Clear all historic steering and trhottle
    for (int i = 0; i < N - 1; i++)
    {
        historic_steering[i] = 0;
        historic_throttle[i] = 0;
    }
}

MPC::~MPC()
{
}

void MPC::InitWithPreviousSolution(Dvector& vars)
{
    for (int i = 0; i < (N - 1) - 1; i++)
    {
        vars[kStateSize + i] = historic_steering[i + 1];
    }
    vars[kStateSize + (N - 1) - 1] = 0;

    for (int i = 0; i < (N - 1) - 1; i++)
    {
        vars[kStateSize + i + N - 1] = historic_throttle[i + 1];
    }
    vars[kStateSize + kActuatorSize * (N - 1) - 1] = 0;
}

void MPC::InitUpperAndLowerBoundsVars(Eigen::VectorXd state, Dvector& vars_upperbound, Dvector& vars_lowerbound)
{
    // First the current state
    for (int i = 0; i < kStateSize; i++)
    {
        vars_lowerbound[i] = state(i);
        vars_upperbound[i] = state(i);
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    const auto kMaxSteeringDelta = 15.0;
    for (int i = kStateSize; i < N - 1 + kStateSize; i++)
    {
        vars_lowerbound[i] = -Deg2Rad(kMaxSteeringDelta);
        vars_upperbound[i] = Deg2Rad(kMaxSteeringDelta);
    }

    for (int i = N - 1 + kStateSize; i < kActuatorSize * (N - 1) + kStateSize; i++)
    {
        vars_lowerbound[i] = 0;
        vars_upperbound[i] = 2;
    }
}

void MPC::InitUpperAndLowerBoundsConstraints(size_t n_constraints,
                                             Dvector& constraints_lowerbound,
                                             Dvector& constraints_upperbound)
{
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = -5 * Pi() / 8;
        constraints_upperbound[i] = 5 * Pi() / 8;
    }
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    size_t i;

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    // N timesteps == N - 1 actuations
    size_t n_vars = kStateSize + (N - 1) * kActuatorSize;
    std::cout << "Number of vars: " << n_vars << endl;

    // Set the number of constraints
    size_t n_constraints = N - 1;

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (int i = 0; i < kStateSize; i++)
    {
        vars[i] = state(i);
    }

    // Initialize to previous solutions, that produces more stable solution
    // Hot-start optimization
    InitWithPreviousSolution(vars);

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Init vars lower and upper bounds
    InitUpperAndLowerBoundsVars(state, vars_upperbound, vars_lowerbound);

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    // Init constraints lower and upper bounds
    InitUpperAndLowerBoundsConstraints(n_constraints, constraints_lowerbound, constraints_upperbound);

    // Object that computes objective and constraints
    int vref, alpha, beta, lambda, nu;
    FgEvaluator fg_eval(coeffs, vref = 20, alpha = 4, beta = 2, lambda = 90, nu = 2);

    // NOTE: You don't have to worry about these options
    // options for IPOPT solver
    std::string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";

    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvaluator>(options,
                                              vars,
                                              vars_lowerbound,
                                              vars_upperbound,
                                              constraints_lowerbound,
                                              constraints_upperbound,
                                              fg_eval,
                                              solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost is evaluated to: " << cost << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.

    // There are cases when the solution imediately jumps to some suboptimal
    // value which is identified by a high cost. In that case, the
    // solution calculated in previous timestamps is incporporated by a "hisotric consideration" factor which aims at
    // smoothing the actual control value.
    vector<double> control;

    double historic_value_factor = 0.25;
    double actual_value_factor = (1 - historic_value_factor);

    auto last_index = N - 1 - 1;
    for (int i = 0; i < last_index; i++)
    {
        historic_steering[i] =
            historic_value_factor * historic_steering[i + 1] + actual_value_factor * solution.x[kStateSize + i];

        historic_throttle[i] =
            historic_value_factor * historic_throttle[i + 1] + actual_value_factor * solution.x[kStateSize + i + N - 1];

        control.push_back(historic_steering[i]);
        control.push_back(historic_throttle[i]);
    }

    historic_steering[last_index] = solution.x[kStateSize + last_index];
    historic_throttle[last_index] = solution.x[kStateSize + kActuatorSize * (N - 1) - 1];

    control.push_back(historic_steering[last_index]);
    control.push_back(historic_throttle[last_index]);

    return control;
}
