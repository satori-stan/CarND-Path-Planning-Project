#include "path_planner.h"

//#include <random>
#include "helpers.h"
#include "ptg.h"

using namespace std;

PathPlanner::PathPlanner(
    const vector<double>& maps_x,
    const vector<double>& maps_y,
    const vector<double>& maps_s,
    const double max_speed,
    const double max_acceleration,
    const double max_jerk) :
    maps_x_(maps_x),
    maps_y_(maps_y),
    maps_s_(maps_s),
    max_speed_(max_speed * 0.95),
    max_acceleration_(max_acceleration),
    max_jerk_(max_jerk),
    target_speed_(0.0),
    current_state_(PlannerStates::kStop),
    current_lane_(99),
    target_lane_(99),
    ptg_(max_speed_, max_acceleration_, max_jerk_, maps_x, maps_y, maps_s) {}

PathPlanner::~PathPlanner() {}

void PathPlanner::operator() (
      const json::value_type& sensor_data,
      vector<double>& new_x,
      vector<double>& new_y) {

  // Main car's localization Data
  double car_x = sensor_data["x"];
  double car_y = sensor_data["y"];
  double car_s = sensor_data["s"];
  double car_d = sensor_data["d"];
  double car_yaw = sensor_data["yaw"];
  double car_speed = sensor_data["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = sensor_data["previous_path_x"];
  auto previous_path_y = sensor_data["previous_path_y"];
  size_t previous_path_size = previous_path_x.size();

  // Previous path's end s and d values 
  double end_path_s = sensor_data["end_path_s"];
  double end_path_d = sensor_data["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = sensor_data["sensor_fusion"];

  // Get our bearings
  vector<double> frenet = Helpers::getFrenet(car_x, car_y,
                                                  car_yaw, maps_x_, maps_y_);
  double current_s = frenet[0];

  if (previous_path_size == 0) {
    end_path_s = car_s;
    end_path_d = car_d;
  }
  current_lane_ = CalculateLane(end_path_d);

  // TODO: Make constant
  double controller_execution_time = 0.02;  // In s

  // TODO: Set the strategy based on our state and our surroundings
  // Figure out possible successor states
  vector<int> possible_successor_states = GetPossibleSuccessorStates();

  PredictAdversariesPositions(controller_execution_time * previous_path_size,
      sensor_fusion);

  ptg_.AssignBase(car_x, car_y, Helpers::deg2rad(car_yaw),
      previous_path_x, previous_path_y);

  // TODO: Calculate the cost of each successor state
  double min_cost = 1E10;
  Trajectory selected_trajectory;
  // max_speed, max_acceleration, max_jerk, last two points
  for (auto possible_state = possible_successor_states.begin();
      possible_state != possible_successor_states.end();
      ++possible_state) {
    // TODO: Generate trajectory for: same speed, faster
    /*
    What changes between possible successor states? The target lane.
    Target speed, accel and jerk are used to generate a trajectory
    but already take into account other cars on the road.
    */
    auto trajectory_for_state = 
        GenerateTrajectory(*possible_state, end_path_s, end_path_d, sensor_fusion);
    /*
    float cost_for_state = calculate_cost(cref_this, predictions,
        trajectory_for_state);
    costs.push_back(cost_for_state);
    */
    if (trajectory_for_state.cost < min_cost) {
      min_cost = trajectory_for_state.cost;
      current_state_ = *possible_state;
      selected_trajectory = trajectory_for_state;
    }
  }
  // TODO: Choose the successor state and path with less cost

  // First we use up all points left over from previous path definition
  // TODO: I am uncomfortable with looping. See if we can directly assign.
  for (size_t i = 0; i < previous_path_size; ++i) {
    new_x.push_back(previous_path_x[i]);
    new_y.push_back(previous_path_y[i]);
  }

  // Now we append the points for the winning trajectory
  for (size_t i = 0; i < 50-previous_path_size; ++i) {
    new_x.push_back(selected_trajectory.x[i]);
    new_y.push_back(selected_trajectory.y[i]);
  }
}

vector<int> PathPlanner::GetPossibleSuccessorStates() {
  vector<int> successors;
  switch (current_state_) {
    case PlannerStates::kStop:
    case PlannerStates::kChangeLaneLeft:
    case PlannerStates::kChangeLaneRight:
      successors.push_back(kKeepLane);
      break;
    case PlannerStates::kKeepLane:
      if (current_lane_ > 1) {
        successors.push_back(kChangeLaneLeft);
        //possible_successor_states.push_back(kPrepareChangeLaneLeft);
      }
      if (current_lane_ < 3) {
        successors.push_back(kChangeLaneRight);
        //possible_successor_states.push_back(kPrepareChangeLaneRight);
      }
      successors.push_back(kKeepLane);
      break;
    /*
    case PlannerStates::kPrepareChangeLaneLeft:
      possible_successor_states.push_back(kPrepareChangeLaneLeft);
      possible_successor_states.push_back(kChangeLaneLeft);
      break;
    case PlannerStates::kPrepareChangeLaneRight:
      possible_successor_states.push_back(kPrepareChangeLaneRight);
      possible_successor_states.push_back(kChangeLaneRight);
      break;
    */
  }
  return successors;
}

void PathPlanner::PredictAdversariesPositions(
    const double seconds_into_the_future,
    nlohmann::json& sensor_fusion) {
  for (size_t i = 0; i < sensor_fusion.size(); ++i) {
    double adversary_vx = sensor_fusion[i][3];
    double adversary_vy = sensor_fusion[i][4];
    double adversary_s = sensor_fusion[i][5];
    double adversary_d = sensor_fusion[i][6];
    double adversary_velocity =
        sqrt(pow(adversary_vx, 2) + pow(adversary_vy, 2));
    double future_adversary_s = adversary_s + (adversary_velocity *
        seconds_into_the_future);
    int adversary_lane = 
    // Now update/insert useful values
    sensor_fusion[i][5] = future_adversary_s;
    sensor_fusion[i][7] = adversary_velocity;
    sensor_fusion[i][8] = adversary_lane;
  }
}

Trajectory PathPlanner::GenerateTrajectory(int possible_state,
    double future_s,
    double future_d,
    nlohmann::json sensor_fusion) {
  switch (possible_state) {
    case PlannerStates::kChangeLaneLeft:
      return {std::vector<double>(), std::vector<double>(), 1E10};
      break;
    case PlannerStates::kChangeLaneRight:
      return {std::vector<double>(), std::vector<double>(), 1E10};
      break;
    case PlannerStates::kKeepLane:
    default:
      return GenerateStraightTrajectory(future_s, future_d, sensor_fusion);
      break;
  }
}

int PathPlanner::CalculateLane(double d) {
  return static_cast<int>(ceil(d / lane_width));
}

Trajectory PathPlanner::GenerateStraightTrajectory(
    double future_s,
    double future_d,
    nlohmann::json sensor_fusion) {
  /*
  random_device rd;
  default_random_engine gen(rd());
  normal_distribution<double> dist_s();
  */
  int starting_lane = static_cast<int>(ceil(future_d / lane_width));
  pair<double, double> boundaries = GetLaneBoundary(starting_lane);
  Trajectory result;
  ptg_.FollowLaneAndLeadingCar(future_s, boundaries.first, boundaries.second,
      sensor_fusion, result);
  return result;
}

pair<double, double> PathPlanner::GetLaneBoundary(int lane) {
  // Leftmost lane is 1
  //int current_lane = static_cast<int>(ceil(car_d / lane_width));
  //double d = (lane - 0.5) * lane_width;
  double lane_start = lane_width * (lane - 1);
  double lane_end = lane_width * lane;
  return {lane_start, lane_end};
}