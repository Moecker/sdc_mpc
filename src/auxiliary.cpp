#include "auxiliary.h"

using CppAD::AD;

double Deg2Rad(double x)
{
    return x * Pi() / 180.0;
}

double Rad2Deg(double x)
{
    return x * 180.0 / Pi();
}

AD<double> Polyeval(Eigen::VectorXd coeffs, AD<double> x)
{
    AD<double> result = 0.0;
    AD<double> pow_value = 1;

    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow_value;
        pow_value *= x;
    }
    return result;
}

Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);

    return result;
}
