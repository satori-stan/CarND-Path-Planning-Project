#ifndef PTG_H_
#define PTG_H_

#include <vector>

#include "json.hpp"

#include "helpers.h"

using json = nlohmann::json;

class PolynomialTrajectoryGenerator {

 public:
  PolynomialTrajectoryGenerator(double max_speed, double max_acceleration,
        double max_jerk);
  virtual ~PolynomialTrajectoryGenerator();

  static void StraightLine(
      const double current_x, const double current_y, const double current_yaw,
      std::vector<double>& new_x, std::vector<double>& new_y);

  void FollowLane(
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      const double current_x,
      const double current_y,
      const double current_yaw,
      std::vector<double>& new_x,
      std::vector<double>& new_y);

  void FollowLaneAndLeadingCar(
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      const json::value_type sensor_data,
      std::vector<double>& new_x,
      std::vector<double>& new_y);

 private:
  
  void CartesianShift(std::vector<double>& x, std::vector<double>& y);
  void CartesianUnshift(std::vector<double>& x, std::vector<double>& y);
  std::vector<double> CartesianUnshift(double x, double y);

  // Data members
  // In mph
  double max_speed_;
  // In m/s^2
  double max_acceleration_;
  // In m/s^3
  double max_jerk_;
  // In mph
  double target_speed_;
  double reference_x_;
  double reference_y_;
  double reference_angle_;
};

#endif  // PTG_H_