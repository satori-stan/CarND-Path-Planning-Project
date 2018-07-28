#ifndef PTG_H_
#define PTG_H_

#include <vector>

#include "json.hpp"

#include "helpers.h"

using json = nlohmann::json;

class PolynomialTrajectoryGenerator {

 public:
  PolynomialTrajectoryGenerator();
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

  // Data members
  // In mph
  double target_speed_;
  //std::vector<double> path_x_;
  //std::vector<double> path_y_;
};

#endif  // PTG_H_