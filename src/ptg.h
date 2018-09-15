#ifndef PTG_H_
#define PTG_H_

#include <vector>

#include "json.hpp"

#include "helpers.h"

using json = nlohmann::json;

struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;
  double cost;
};

struct SingleAxisTrajectoryBase {
  double velocity;
  double acceleration;
  double jerk;
  double cost;
};

class PolynomialTrajectoryGenerator {

 public:
  PolynomialTrajectoryGenerator(const double max_speed, const double max_acceleration,
      const double max_jerk,
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s);
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
      /*
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      */
      const double car_s,
      const double lane_start,
      const double lane_end,
      const json sensor_fusion,
      /*
      std::vector<double>& new_x,
      std::vector<double>& new_y
      */
     Trajectory& out);

  void Generate(
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      const json::value_type sensor_data,
      std::vector<double>& new_x,
      std::vector<double>& new_y);

  void AssignBase(const double current_x,
                  const double current_y,
                  const double current_theta,
                  const std::vector<double>& previous_path_x,
                  const std::vector<double>& previous_path_y);

 private:
  
  const double kControllerExecutionTime = 0.02;  // In s  // TODO: Avoid duplication
  const size_t kPathPoints = 50;

  void CartesianShift(std::vector<double>& x, std::vector<double>& y);
  void CartesianUnshift(std::vector<double>& x, std::vector<double>& y);
  std::vector<double> CartesianUnshift(double x, double y);

  // Data members
  // In mph
  double max_speed_;
  // In m/s
  double max_velocity_;
  // In m/s^2
  double max_acceleration_;
  // In m/s^3
  double max_jerk_;
  // In mph
  double target_speed_;
  double reference_x_;
  double reference_y_;
  double reference_angle_;

  double current_velocity_;
  double current_acceleration_;
  double current_jerk_;
  int current_lane_;
  size_t previous_path_size_;

  std::vector<double> maps_x_;
  std::vector<double> maps_y_;
  std::vector<double> maps_s_;

  std::vector<double> x_;
  std::vector<double> y_;
};

#endif  // PTG_H_