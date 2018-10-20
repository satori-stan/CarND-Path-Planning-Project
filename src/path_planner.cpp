#include "path_planner.h"

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
    max_acceleration_(max_acceleration * 0.80),
    max_jerk_(max_jerk * 0.80),
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
  vector<Trajectory> possible_trajectories;

  for (auto possible_state = possible_successor_states.begin();
      possible_state != possible_successor_states.end();
      ++possible_state) {
    /*
    What changes between possible successor states? The target lane.
    Target speed, accel and jerk are used to generate a trajectory
    but already take into account other cars on the road.
    */
    GenerateTrajectory(*possible_state, possible_trajectories, end_path_s,
        end_path_d, sensor_fusion);
  }
  printf("\n");

  // Choose the successor state and path with less cost
  double min_cost = 1E10;
  Trajectory selected_trajectory;

  for (auto trajectory = possible_trajectories.begin();
      trajectory != possible_trajectories.end();
      ++trajectory) {

    //printf("%i %f\t", trajectory->state, trajectory->cost);
    if (trajectory->cost < min_cost) {
      min_cost = trajectory->cost;
      current_state_ = trajectory->state;
      selected_trajectory = *trajectory;
      ptg_.current_velocity_ = trajectory->s_velocity;
      ptg_.current_acceleration_ = trajectory->s_acceleration;
      ptg_.current_normal_acceleration_ = trajectory->d_acceleration;
      // TODO: Update the current lane if we end up changing
      current_lane_ = trajectory->lane;
    }
  }
  printf("\nCurrent state: %i\n", current_state_);

  // First we use up all points left over from previous path definition
  // TODO: I am uncomfortable with looping. See if we can directly assign.
  for (size_t i = 0; i < previous_path_size; ++i) {
    new_x.push_back(previous_path_x[i]);
    new_y.push_back(previous_path_y[i]);
  }

  // Now we append the points for the winning trajectory
  for (size_t i = 0; i < selected_trajectory.x.size(); ++i) {
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
    //double adversary_s = sensor_fusion[i][5] % 6945.554;
    double adversary_d = sensor_fusion[i][6];
    double adversary_velocity =
        sqrt(pow(adversary_vx, 2) + pow(adversary_vy, 2));
    double future_adversary_s = adversary_s + (adversary_velocity *
        seconds_into_the_future);
    int adversary_lane = CalculateLane(adversary_d);
    // Now update/insert useful values
    sensor_fusion[i][5] = future_adversary_s;
    sensor_fusion[i][7] = adversary_velocity;
    sensor_fusion[i][8] = adversary_lane;
    printf("%f\t%f\t%f\t%i\n", future_adversary_s, adversary_d,
        adversary_velocity, adversary_lane);
  }
  printf("\n");
}

void PathPlanner::GenerateTrajectory(int possible_state,
    vector<Trajectory>& possible_trajectories,
    double future_s,
    double future_d,
    nlohmann::json sensor_fusion) {

  double target_d = future_d;
  int target_lane = CalculateLane(target_d);
  pair<double, double> target_lane_boundaries;
  size_t trajectories = possible_trajectories.size();

  switch (possible_state) {
    case PlannerStates::kChangeLaneLeft:
      target_lane -= 1;
      target_lane_boundaries = GetLaneBoundary(target_lane);
      target_d =
          (target_lane_boundaries.first + target_lane_boundaries.second) / 2.0;
      ptg_.ChangeLane(future_s, future_d, target_d, sensor_fusion,
          possible_trajectories);
      break;

    case PlannerStates::kChangeLaneRight:
      target_lane += 1;
      target_lane_boundaries = GetLaneBoundary(target_lane);
      target_d =
          (target_lane_boundaries.first + target_lane_boundaries.second) / 2.0;
      ptg_.ChangeLane(future_s, future_d, target_d, sensor_fusion,
          possible_trajectories);
      break;

    case PlannerStates::kKeepLane:
    default:
      target_lane_boundaries = GetLaneBoundary(target_lane);
      target_d =
          (target_lane_boundaries.first + target_lane_boundaries.second) / 2.0;
      ptg_.FollowLaneAndLeadingCar(future_s, future_d,
          target_lane_boundaries.first, target_lane_boundaries.second,
          sensor_fusion, possible_trajectories);
      break;
  }

  for (auto trajectory = possible_trajectories.begin() + trajectories;
      trajectory != possible_trajectories.end();
      ++trajectory) {

    trajectory->state = possible_state;
    trajectory->lane = target_lane;
    CalculateCost(*trajectory, sensor_fusion, future_s, future_d,
        target_lane_boundaries.first, target_lane_boundaries.second);
  }
}

int PathPlanner::CalculateLane(double d) {
  return static_cast<int>(ceil(d / kLaneWidth));
}

pair<double, double> PathPlanner::GetLaneBoundary(int lane) {
  // Leftmost lane is 1
  double lane_start = kLaneWidth * (lane - 1);
  double lane_end = kLaneWidth * lane;
  return {lane_start, lane_end};
}

void PathPlanner::CalculateCost(Trajectory& option,
    nlohmann::json sensor_fusion,
    double current_s,
    double current_d,
    double target_lane_start,
    double target_lane_end) {

  double max_velocity = max_speed_ * 1.609344 * 1000 / 3600;
  printf("%u(%u-%u): #[%u] s[%.2f] d[%.2f] s_dot[%.2f] s_dot2[%.2f] "
      "s_dot3[%.2f] d_dot[%.2f] d_dot2[%.2f] avg_d_dot[%.2f] ",
      option.state,
      current_lane_,
      option.lane,
      option.total_size,
      option.s,
      option.d,
      option.s_velocity,
      option.s_acceleration,
      option.s_jerk,
      option.d_velocity,
      option.d_acceleration,
      option.avg_d_acceleration);

  // The cost of plotting too much into the future
  double expected_points = Helpers::UnsignedToSigned(kPathPoints) * 1.0;
  option.cost += Helpers::Logistic(
      (Helpers::UnsignedToSigned(option.total_size) - expected_points) /
      expected_points) * 3.0;

  // Cost of not moving
  option.cost += (option.s_velocity <= 0.0 ? 1.0 : 0.0) * 100.0;

  // Cost of going in reverse
  option.cost += (option.s_velocity < 0.0 ? 1.0 : 0.0) * 50.0;

  // Cost of going too slow
  option.cost += (max((max_velocity - option.s_velocity) /
      (option.s_velocity + 1.0), 0.0) / max_velocity) * 250.0;

  // Cost of breaking
  option.cost += (option.s_acceleration + abs(option.d_acceleration) < 0.0 ?
      Helpers::Logistic(sqrt(pow(option.s_acceleration, 2) +
      pow(option.d_acceleration, 2)) / max_acceleration_) : 0.0) * 0.0;

  // Cost of going over the speed limit
  option.cost += (option.s_velocity > max_velocity ? 1.0 : 0.0) * 30.0;

  // Cost of going over the acceleration limit
  option.cost += (option.s_acceleration > max_acceleration_ ? 1.0 : 0.0) * 40.0;

  // Cost of high acceleration
  option.cost += (abs(option.s_acceleration) / max_acceleration_) * 0.0;

  // The cost of high jerk
  option.cost += Helpers::Logistic(option.s_jerk / max_jerk_) * 0.0;

  // The cost of going over the maximum jerk
  option.cost += (option.s_jerk > max_jerk_ ? 1.0 : 0.0) * 1.0;

  // The cost of ending up too far from the middle of the lane
  option.cost += Helpers::Logistic(abs(option.d -
      (target_lane_start + target_lane_end) / 2.0) / (kLaneWidth / 2.0)) * 0.0;

  // The cost of not reaching the target lane
  option.cost += (option.d < target_lane_start || option.d > target_lane_end ?
      1 : 0) * 23.0;

  // Cost of changing lanes too fast
  option.cost +=
      Helpers::Logistic(option.d_acceleration / max_acceleration_) * 0.0;

  // Cost of going out of bounds
  option.cost += ((option.lane < 1 || option.lane > 3) ? 1.0 : 0.0) * 80;

  printf("C1[%.4f] ", option.cost);

  int target_lane = option.lane;
  double distance_to_closest_leading_car = 1E9;
  double distance_to_current_leading_car = 1E9;
  double distance_to_current_following_car = -1E9;
  double distance_to_future_leading_car = 1E9;
  double distance_to_future_following_car = -1E9;
  double current_leading_car_s_velocity = 100.0;
  double current_following_car_s_velocity = 100.0;
  double future_leading_car_s_velocity = 100.0;
  double future_following_car_s_velocity = 100.0;
  unsigned int* leading_current = nullptr;
  unsigned int* following_future = nullptr;

  for (size_t i = 0; i < sensor_fusion.size(); ++i) {
    /*
      Map of adversary cars in sensor fusion array
      TODO: Use enum
      0 unique ID
      1 x position in map coordinates
      2 y position in map coordinates
      3 x velocity in m/s
      4 y velocity in m/s
      5 s position in frenet coordinates
      6 d position in frenet coordinates
      7 velocity magnitude
      8 lane
    */
    unsigned int adversary = sensor_fusion[i][0];
    double adversary_s = sensor_fusion[i][5];
    double adversary_d = sensor_fusion[i][6];
    double adversary_velocity = sensor_fusion[i][7];
    int adversary_lane = sensor_fusion[i][8];

    // Consider only cars which are in front of us or "right" behind us.
    if (adversary_lane == target_lane) {
      // TODO: Account for cars that are moving into our lane!
      // TODO: Beware, the s value wraps around
      double s_distance_from_current = adversary_s - current_s;

      // It is the closest car in front of where we are
      if (adversary_s > current_s &&
          s_distance_from_current < distance_to_current_leading_car) {
        distance_to_current_leading_car = s_distance_from_current;
        current_leading_car_s_velocity = adversary_velocity;
        leading_current = &adversary;  // XXX: This should be safe, right?
      }

      if (adversary_s <= current_s &&
          s_distance_from_current > distance_to_current_following_car) {
        distance_to_current_following_car = s_distance_from_current;
        current_following_car_s_velocity = adversary_velocity;
      }

      // TODO: Should project the aversary's position in time, at least for
      //       those options which require more points!
      double s_distance_from_target = adversary_s - option.s;
      // It is the closest car in front of where we will be
      if (adversary_s > option.s &&
          s_distance_from_target < distance_to_future_leading_car) {
        distance_to_future_leading_car = s_distance_from_target;
        future_leading_car_s_velocity = adversary_velocity;
      }

      if (adversary_s <= option.s &&
          s_distance_from_target > distance_to_future_following_car) {
        distance_to_future_following_car = s_distance_from_target;
        future_following_car_s_velocity = adversary_velocity;
        following_future = &adversary;
      }
    } else if (adversary_lane == current_lane_) {
      double s_distance_from_current = adversary_s - current_s; // option.s;
      if (adversary_s > current_s &&
          s_distance_from_current < distance_to_closest_leading_car) {
        distance_to_closest_leading_car = s_distance_from_current;
      }
    }
  }
  // Distance buffer
  option.cost += (max((kSafetyDistance - distance_to_future_leading_car) /
      (distance_to_future_leading_car + 1), 0.0) / kSafetyDistance) * 100.0;

  // Unnecessary lane change
  option.cost += (option.lane != current_lane_ &&
      distance_to_closest_leading_car > kSafetyDistance ? 1 : 0) * 120.0;

  double collision_time_buffer = 10.0;
  double multiplier = current_lane_ != target_lane ? 1.0 : 1.0;
  // TODO: Promote to function
  // Buffer leading car
  {
    double velocity_delta = option.s_velocity - current_leading_car_s_velocity;
    double time_to_collision = distance_to_current_leading_car / velocity_delta;
    option.cost += max(1.0 - ((time_to_collision > 0.0 ? time_to_collision :
        collision_time_buffer) / collision_time_buffer), 0.0) * multiplier * 0.0;
  }

  // Buffer following car
  {
    double velocity_delta = option.s_velocity - current_following_car_s_velocity;
    double time_to_collision = distance_to_current_following_car / velocity_delta;
    option.cost += max(1.0 - ((time_to_collision > 0.0 ? time_to_collision :
        collision_time_buffer) / collision_time_buffer), 0.0) * multiplier * 10.0;
  }

  // Buffer target leading car
  {
    double velocity_delta = option.s_velocity - future_leading_car_s_velocity;
    double time_to_collision = distance_to_future_leading_car / velocity_delta;
    option.cost += max(1.0 - ((time_to_collision > 0.0 ? time_to_collision :
        collision_time_buffer) / collision_time_buffer), 0.0) * multiplier * 70.0;
  }

  // Buffer target following car
  {
    double velocity_delta = option.s_velocity - future_following_car_s_velocity;
    double time_to_collision = distance_to_future_following_car / velocity_delta;
    option.cost += max(1.0 - ((time_to_collision > 0.0 ? time_to_collision :
        collision_time_buffer) / collision_time_buffer), 0.0) * multiplier * 0.0;
  }

  // Collision
  double car_radius = 5.0;  // TODO: Either make constant or a parameter
  if (abs(distance_to_current_leading_car) < car_radius ||
      abs(distance_to_future_leading_car) < car_radius) {
    option.cost += 30.0;
  }

  // Rear collision
  if (abs(distance_to_current_following_car) < car_radius ||
      abs(distance_to_future_following_car) < car_radius) {
    option.cost += 60.0;
  }

  // Side collision
  if (leading_current && following_future &&
      *leading_current == *following_future) {
    option.cost += 2.0;
  }

  printf("C2[%.4f]", option.cost);
  printf("\n");
}