#ifndef FG_EVAL_H
#define FG_EVAL_H

#include "Eigen-3.3/Eigen/Core"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

class FgEvaluator
{
  public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    FgEvaluator(Eigen::VectorXd coeffs, double v, double a, double b, double l, double n)
            : v_reference_(v), alpha_(a), beta_(b), lambda_(l), nu_(n)
    {
        this->coeffs = coeffs;
    }

    void operator()(ADvector& fg, const ADvector& vars);

    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

  private:
    double v_reference_;
    double alpha_;
    double beta_;
    double lambda_;
    double nu_;
};

#endif /* FG_EVAL_H */
