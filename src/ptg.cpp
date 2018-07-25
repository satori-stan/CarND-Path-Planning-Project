#include "ptg.h"

#include "spline.h"

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

void PolynomialTrajectoryGenerator::FollowLane(const double current_x,
      const double current_y, const double current_yaw, 
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      std::vector<double>& x,
      std::vector<double>& y) {
  
  std::vector<double> frenet = Helpers::getFrenet(current_x, current_y,
                                                  current_yaw, maps_x, maps_y);
  double current_s = frenet[0];
  double current_d = 6;  // Should use frenet[1], but the sparsity of the map
                         // data can cause the value to vary.

  // 50 points into the future, but only move in s coordinate
  double distance_increment = 0.4; // 0.5 is too fast!
  for (size_t i = 1; i <= 50; ++i) {
    double delta = distance_increment * i;
    double s = current_s + delta;
    double d = current_d;
    std::vector<double> cartesian = Helpers::getXY(s, d, maps_s, maps_x, maps_y);
    x.push_back(cartesian[0]);
    y.push_back(cartesian[1]);
  }
}