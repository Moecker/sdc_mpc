#include <math.h>

#include "coordinate_systems.h"

void CoordinateSystems::SetPosition(double x, double y, double phi)
{
    x_ = x;
    y_ = y;
    phi_ = phi;
}

void CoordinateSystems::Transform(const double global_x, const double global_y, double& local_x, double& local_y)
{
    local_x = cos(phi_) * (global_x - x_) + sin(phi_) * (global_y - y_);
    local_y = -sin(phi_) * (global_x - x_) + cos(phi_) * (global_y - y_);
}

void CoordinateSystems::Transform(const std::vector<double> global_x,
                                  const std::vector<double> global_y,
                                  std::vector<double>& local_x,
                                  std::vector<double>& local_y)
{
    local_x.resize(global_x.size());
    local_y.resize(global_y.size());

    for (int i = 0; i < global_x.size(); i++)
    {
        Transform(global_x[i], global_y[i], local_x[i], local_y[i]);
    }
}

void CoordinateSystems::InverseTransform(const double local_x, const double local_y, double& global_x, double& global_y)
{
    global_x = x_ + cos(phi_) * local_x - sin(phi_) * local_y;
    global_y = y_ + sin(phi_) * local_x + cos(phi_) * local_y;
}

void CoordinateSystems::InverseTransform(const std::vector<double> local_x,
                                         const std::vector<double> local_y,
                                         std::vector<double>& global_x,
                                         std::vector<double>& global_y)
{
    global_x.resize(local_x.size());
    global_y.resize(local_y.size());

    for (int i = 0; i < local_x.size(); i++)
    {
        InverseTransform(local_x[i], local_y[i], global_x[i], global_y[i]);
    }
}
