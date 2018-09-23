#ifndef PTG_H_
#define PTG_H_

#include <vector>

#include "json.hpp"

#include "helpers.h"

using json = nlohmann::json;

const size_t kPathPoints = 50;

struct Trajectory {
  int state;
  double cost;
  double s_velocity;
  double s_acceleration;
  double d_velocity;
  double d_acceleration;
  std::vector<double> x;
  std::vector<double> y;
};

struct SingleAxisTrajectoryBase {
  double velocity;
  double acceleration;
  double jerk;
  double cost;
};

struct TwoAxesTrajectoryBase {
  double s_velocity;
  double s_acceleration;
  double s_jerk;
  double d_velocity;
  double d_acceleration;
  double d_jerk;
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
      const double car_s,
      const double lane_start,
      const double lane_end,
      const json sensor_fusion,
      Trajectory& out);

  void ChangeLane(
      const double car_s,
      const double current_d,
      const double target_d,
      const json sensor_fusion,
      std::vector<Trajectory>& out);

  void AssignBase(const double current_x,
                  const double current_y,
                  const double current_theta,
                  const std::vector<double>& previous_path_x,
                  const std::vector<double>& previous_path_y);

 private:
  
  const double kControllerExecutionTime = 0.02;  // In s  // TODO: Avoid duplication

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
  // In global coordinates
  double reference_x_;
  double reference_y_;
  double reference_angle_;

  double current_velocity_;
  double current_acceleration_;
  double current_jerk_;
  size_t previous_path_size_;

  std::vector<double> maps_x_;
  std::vector<double> maps_y_;
  std::vector<double> maps_s_;

  std::vector<double> x_;
  std::vector<double> y_;
};

#endif  // PTG_H_