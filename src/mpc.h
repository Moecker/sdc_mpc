#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <vector>

#include "Eigen-3.3/Eigen/Core"

extern const double Lf;
extern const double dt;

using namespace std;

class MPC
{
  public:
    typedef CPPAD_TESTVECTOR(double) Dvector;

    static constexpr size_t N = 8;
    static constexpr double dt = 0.5;

    static const int kStateSize = 4;
    static const int kActuatorSize = 2;

    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    static constexpr double Lf = 2.67;

    MPC();
    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    void InitWithPreviousSolution(Dvector& vars);
    void InitUpperAndLowerBoundsVars(Eigen::VectorXd state, Dvector& vars_upperbound, Dvector& vars_lowerbound);
    void InitUpperAndLowerBoundsConstraints(size_t n_constraints,
                                            Dvector& constraints_lowerbound,
                                            Dvector& constraints_upperbound);

  private:
    vector<double> historic_steering;
    vector<double> historic_throttle;
};

#endif /* MPC_H */
