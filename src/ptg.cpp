#include "ptg.h"

void PolynomialTrajectoryGenerator::StraightLine(const double current_x,
      const double current_y, const double current_yaw, std::vector<double>& x,
      std::vector<double>& y) {
  // 50 points into the future
  double distance_increment = 0.5;
  for (size_t i = 0; i < 50; ++i) {
    double delta = distance_increment * i;
    x.push_back(current_x + delta * cos(Helpers::deg2rad(current_yaw)));
    y.push_back(current_y);
  }
}