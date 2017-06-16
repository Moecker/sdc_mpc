#ifndef MPC_COORDINATESYSTEMS_H
#define MPC_COORDINATESYSTEMS_H

#include <vector>

class CoordinateSystems
{
  public:
    CoordinateSystems(double x, double y, double phi) : x_(x), y_(y), phi_(phi) {}

    void SetPosition(double x, double y, double phi);

    void Transform(const double global_x, const double global_y, double& local_x, double& local_y);
    void Transform(const std::vector<double> global_x,
                   const std::vector<double> global_y,
                   std::vector<double>& local_x,
                   std::vector<double>& local_y);

    void InverseTransform(const double local_x, const double local_y, double& global_x, double& global_y);
    void InverseTransform(const std::vector<double> local_x,
                          const std::vector<double> local_y,
                          std::vector<double>& global_x,
                          std::vector<double>& global_y);

  private:
    double phi_;
    double x_;
    double y_;
};

#endif  // MPC_COORDINATESYSTEMS_H
