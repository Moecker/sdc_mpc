#ifndef FG_EVAL_H
#define FG_EVAL_H

#include "Eigen-3.3/Eigen/Core"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

class FgEvaluator
{
  public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    FgEvaluator(Eigen::VectorXd coeffs,
                double v_reference,
                double w_position,
                double w_speed,
                double w_steering,
                double w_trottle)
            : v_reference_(v_reference),
              position_weight_(w_position),
              speed_weight_(w_speed),
              steering_weight_(w_steering),
              throttle_weight_(w_trottle)
    {
        this->coeffs = coeffs;
    }

    void operator()(ADvector& fg, const ADvector& vars);

    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

  private:
    double v_reference_;

    // Certain weights to vary impacts of certain cost contributions
    double position_weight_;
    double speed_weight_;
    double steering_weight_;
    double throttle_weight_;
};

#endif /* FG_EVAL_H */
