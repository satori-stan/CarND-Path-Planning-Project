#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>

#include "json.hpp"

#include "ptg.h"

using json = nlohmann::json;

class PathPlanner {

 public:

  PathPlanner(
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      const double max_speed,
      const double max_acceleration,
      const double max_jerk);

  virtual ~PathPlanner();

  void operator() (
      const json::value_type& sensor_data,
      std::vector<double>& new_x,
      std::vector<double>& new_y);

 private:

  enum PlannerStates {
    kStop,
    kKeepLane,
    kPrepareChangeLaneRight,
    kPrepareChangeLaneLeft,
    kChangeLaneRight,
    kChangeLaneLeft
  };

  // TODO: This is a property of the road we are on, not a constant but sparsely
  //       changed.
  const double lane_width = 4;  // In m

  std::vector<int> GetPossibleSuccessorStates();

  void PredictAdversariesPositions(const double seconds_into_the_future,
      nlohmann::json& sensor_fusion);

  Trajectory GenerateTrajectory(
      int possible_state,
      double future_s,
      double future_d,
      nlohmann::json predicted_sensor_fusion);

  Trajectory GenerateStraightTrajectory(
      double future_s,
      double future_d,
      nlohmann::json sensor_fusion);

  int CalculateLane(double d);
  std::pair<double, double> GetLaneBoundary(int lane);

  const std::vector<double>& maps_x_;
  const std::vector<double>& maps_y_;
  const std::vector<double>& maps_s_;

  // In mph
  double max_speed_;
  // In m/s^2
  double max_acceleration_;
  // In m/s^3
  double max_jerk_;
  // In mph
  double target_speed_;

  int current_state_;
  int current_lane_;
  int target_lane_;

  PolynomialTrajectoryGenerator ptg_;
};

#endif  // PATH_PLANNER_H_