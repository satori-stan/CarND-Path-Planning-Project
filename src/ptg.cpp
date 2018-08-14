#include "ptg.h"

#include <iostream>

#include "spline.h"

using namespace std;

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator(
    double max_speed,
    double max_acceleration,
    double max_jerk,
    const vector<double>& maps_x,
    const vector<double>& maps_y,
    const vector<double>& maps_s) :
  max_speed_(max_speed),
  max_acceleration_(max_acceleration),
  max_jerk_(max_jerk),
  target_speed_(0),
  current_lane_(-1),
  maps_x_(maps_x),
  maps_y_ (maps_y),
  maps_s_(maps_s) {}

PolynomialTrajectoryGenerator::~PolynomialTrajectoryGenerator() {}

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

void PolynomialTrajectoryGenerator::FollowLane(
      const std::vector<double>& maps_x,
      const std::vector<double>& maps_y,
      const std::vector<double>& maps_s,
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      const double current_x,
      const double current_y,
      const double current_yaw, 
      std::vector<double>& next_x,
      std::vector<double>& next_y) {
  
  std::vector<double> frenet = Helpers::getFrenet(current_x, current_y,
                                                  current_yaw, maps_x, maps_y);
  double current_s = frenet[0];
  // Should use frenet[1], but the sparsity of the map data can cause the value
  // to vary. Instead we express the value of d in terms of the lane we are
  // keeping.
  double lane_width = 4;  // In m
  int current_lane = 2;
  double current_d = (current_lane - 0.5) * lane_width;

  // 50 points into the future, but only move in s coordinate
  size_t path_points = 50;
  double target_speed = 49.5;  // In mph
  // mph * km_per_mile * meters_per_km / seconds_per_hour
  double target_velocity = target_speed * 1.609344 * 1000 / 3600;  // In m/s
  double controller_execution_time = 0.02;  // In s
  double distance_increment = controller_execution_time * target_velocity;  // path_points;

  // We build a path buffer to make sure the trajectory is always smooth
  std::vector<double> x;
  std::vector<double> y;
  size_t previous_path_size = previous_path_x.size();
  double first_x;
  if (previous_path_size < 2) {
    // If we have less than two points left over from our last path calculation,
    // we approximate two points with the current position and 1 unit in the
    // past based on the current yaw.
    first_x = current_x - cos(Helpers::deg2rad(current_yaw));
    x.push_back(first_x);
    y.push_back(current_y - sin(Helpers::deg2rad(current_yaw)));

    x.push_back(current_x);
    y.push_back(current_y);
  } else {
    // If we have more than two points left over from our last path calculation,
    // grab the first two.
    first_x = previous_path_x[0];
    x.push_back(previous_path_x[0]);
    y.push_back(previous_path_y[0]);

    x.push_back(previous_path_x[1]);
    y.push_back(previous_path_y[1]);
  }

  // Choose (two?) points ahead to draw a line. Thinking of a similar distance
  // increment, distance_increment * 25 and distance_increment * 50.
  std::vector<double> cartesian;
  cartesian = Helpers::getXY(current_s + 25, current_d, maps_s, maps_x, maps_y);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  cartesian = Helpers::getXY(current_s + 50, current_d, maps_s, maps_x, maps_y);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  double last_x = cartesian[0];

  // Now we use a spline (from http://kluge.in-chemnitz.de/opensource/spline/)
  // to plot a smooth trajectory that touches all our points.
  tk::spline s;
  s.set_points(x, y);  // Set must be sorted by x ascending!!
  // new_y = s(new_x);

  //double distance_increment = (last_x - first_x) / 50;

  for (size_t i = 1; i <= 50; ++i) {
    double delta = distance_increment * i;
    double x = current_x + delta;
    next_x.push_back(x);
    next_y.push_back(s(x));
  }
}

void PolynomialTrajectoryGenerator::FollowLaneAndLeadingCar(
    const double car_s,
    const double lane_start,
    const double lane_end,
    const json sensor_fusion,
    /*
    vector<double>& next_x,
    vector<double>& next_y
    */
    Trajectory& out) {

  // Starting cost value
  out.cost = 0;

  // Make sure we slow down if there is a car in our path
  double controller_execution_time = 0.02;  // In s  // TODO: Avoid duplication
  bool too_close_to_leading_car = false;
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
    /*
    double adversary_vx = sensor_fusion[i][3];
    double adversary_vy = sensor_fusion[i][4];
    */
    double adversary_s = sensor_fusion[i][5];
    double adversary_d = sensor_fusion[i][6];
    if (adversary_d > lane_start && adversary_d < lane_end) {
      // The other car is in our lane!
      // TODO: Beware, the s value wraps around
      // TODO: Make vehicle safety distance into a variable
      // We figure out where it'll be with respect to us (in the future)
      if (adversary_s > car_s && (adversary_s-car_s) < 30) {
        // We are too close!
        too_close_to_leading_car = true;
        break;  // No need to check all the rest of the cars
      }
    }
  }

  // TODO: Make increments and decrements proportional to the acceleration limits.
  // TODO: Another option is to just match the speed of the car infront of us
  if (too_close_to_leading_car) {
    target_speed_ -= 0.2;
  } else if (target_speed_ < max_speed_) {
    target_speed_ += 0.2;
  }

  out.cost = 1;  // TODO: (Desired speed - target speed) * factor / Desired speed

  // mph * km_per_mile * meters_per_km / seconds_per_hour
  double target_velocity = target_speed_ * 1.609344 * 1000 / 3600;  // In m/s
  double distance_increment = controller_execution_time * target_velocity;  // path_points;
  // Figure out a decent number of points that will let us plan 20m into the
  // future.
  size_t path_points = min(static_cast<size_t>(100),
      static_cast<size_t>((20 / distance_increment) + 0.5));

  double target_d = (lane_end + lane_start) / 2.0;

  vector<double> x(x_);
  vector<double> y(y_);
  // Choose (two?) points ahead to draw a line. Thinking of a similar distance
  // increment, distance_increment * 25 and distance_increment * 50.
  vector<double> cartesian;
  cartesian = Helpers::getXY(car_s + 25, target_d, maps_s_, maps_x_, maps_y_);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  cartesian = Helpers::getXY(car_s + 50, target_d, maps_s_, maps_x_, maps_y_);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  // Now shift points to use the car's reference frame
  CartesianShift(x, y);

  // Now we use a spline (from http://kluge.in-chemnitz.de/opensource/spline/)
  // to plot a smooth trajectory that touches all our points.
  tk::spline s;
  s.set_points(x, y);  // Set must be sorted by x ascending!!

  // Then we create new points from the spline
  for (size_t i = 1; i <= 50; ++i) {
    double delta = distance_increment * i;
    // TODO: Improve next line, it seems wasteful.
    vector<double> calculated = CartesianUnshift(delta, s(delta));
    out.x.push_back(calculated[0]);
    out.y.push_back(calculated[1]);
  }
}

void PolynomialTrajectoryGenerator::AssignBase(
    const double current_x,
    const double current_y,
    const double current_theta,
    const vector<double>& previous_x,
    const vector<double>& previous_y) {
  size_t previous_path_size = previous_x.size();
  x_.clear();
  y_.clear();
  if (previous_path_size < 2) {
    reference_x_ = current_x;
    reference_y_ = current_y;
    reference_angle_ = current_theta;

    x_.push_back(reference_x_ - cos(reference_angle_));
    y_.push_back(reference_y_ - sin(reference_angle_));

    x_.push_back(reference_x_);
    y_.push_back(reference_y_);

  } else {
    double prev_reference_x = previous_x[previous_path_size - 2];
    double prev_reference_y = previous_y[previous_path_size - 2];
    x_.push_back(prev_reference_x);
    y_.push_back(prev_reference_y);

    reference_x_ = previous_x[previous_path_size - 1];
    reference_y_ = previous_y[previous_path_size - 1];
    x_.push_back(reference_x_);
    y_.push_back(reference_y_);

    reference_angle_ = atan2(reference_y_ - prev_reference_y,
                             reference_x_ - prev_reference_x);
  }
}

void PolynomialTrajectoryGenerator::Generate(
    const std::vector<double>& maps_x,
    const std::vector<double>& maps_y,
    const std::vector<double>& maps_s,
    json::value_type sensor_data,
    std::vector<double>& next_x,
    std::vector<double>& next_y) {
  
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
  std::vector<double> frenet = Helpers::getFrenet(car_x, car_y,
                                                  car_yaw, maps_x, maps_y);
  double current_s = frenet[0];
  // TODO: This is a property of the road we are on, not a constant but sparsely
  //       changed.
  double lane_width = 4;  // In m
  // Leftmost lane is 1
  int current_lane = static_cast<int>(ceil(car_d / lane_width));
  double current_d = (current_lane - 0.5) * lane_width;
  double current_lane_start = lane_width * (current_lane - 1);
  double current_lane_end = lane_width * current_lane;

  // TODO: Make constant
  double controller_execution_time = 0.02;  // In s

  // TODO: Come up with different trajectories
  // Make sure we slow down if there is a car in our path
  bool too_close_to_leading_car = false;
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
    */
    double adversary_vx = sensor_fusion[i][3];
    double adversary_vy = sensor_fusion[i][4];
    double adversary_s = sensor_fusion[i][5];
    double adversary_d = sensor_fusion[i][6];
    if (adversary_d > current_lane_start && adversary_d < current_lane_end) {
      // The other car is in our lane!
      // TODO: Beware, the s value wraps around
      // TODO: Make vehicle safety distance into a variable
      // We figure out where it'll be with respect to us (in the future)
      double adversary_velocity =
          sqrt(pow(adversary_vx, 2) + pow(adversary_vy, 2));
      double future_adversary_s = adversary_s + controller_execution_time *
          adversary_velocity * previous_path_size;
      if (future_adversary_s > end_path_s && (future_adversary_s-end_path_s) < 30) {
        // We are too close!
        too_close_to_leading_car = true;
        break;  // No need to check all the rest of the cars
      }
    }
  }

  int target_lane = current_lane;

  // TODO: Make increments and decrements proportional to the acceleration limits.
  if (too_close_to_leading_car) {
    // TODO: Change min and max lane numbers to variables
    if (current_lane > 1) {
      target_lane = current_lane - 1;
    } else if (current_lane < 3) {
      target_lane = current_lane + 1;
    } else {
      target_speed_ -= 0.2;
    }
  } else if (target_speed_ < max_speed_) {
    target_speed_ += 0.2;
  }

  if (target_lane != current_lane) {
    std::cout << target_lane << std::endl;
  }
  double target_d = (target_lane - 0.5) * lane_width;
  //double target_lane_start = lane_width * (target_lane - 1);
  //double target_lane_end = lane_width * target_lane;

  // mph * km_per_mile * meters_per_km / seconds_per_hour
  double target_velocity = target_speed_ * 1.609344 * 1000 / 3600;  // In m/s
  double distance_increment = controller_execution_time * target_velocity;  // path_points;
  // Figure out a decent number of points that will let us plan 20m into the
  // future.
  size_t path_points = min(static_cast<size_t>(100),
      static_cast<size_t>((20 / distance_increment) + 0.5));

  // We build a path buffer to make sure the trajectory is always smooth
  std::vector<double> x;
  std::vector<double> y;
  double first_x;
  // Rotate/translate coordinates to simplify calculations when movement
  // is mostly in y coordinate.
  reference_x_ = car_x;
  reference_y_ = car_y;
  reference_angle_ = Helpers::deg2rad(car_yaw);

  // Use more points from the previous path to account for lag between
  // path calculation and actual position upon execution.
  if (previous_path_size < 2) {
    // If we have less than two points left over from our last path calculation,
    // we approximate two points with the current position and 1 unit in the
    // past based on the current yaw.
    first_x = reference_x_ - cos(reference_angle_);
    x.push_back(first_x);
    y.push_back(reference_y_ - sin(reference_angle_));

    x.push_back(reference_x_);
    y.push_back(reference_y_);
  } else {
    // Don't use too much of the previous path, to allow room to maneuver.
    //previous_path_size = min(static_cast<size_t>(20), previous_path_size);

    double prev_reference_x = previous_path_x[previous_path_size - 2];
    double prev_reference_y = previous_path_y[previous_path_size - 2];
    x.push_back(prev_reference_x);
    y.push_back(prev_reference_y);

    reference_x_ = previous_path_x[previous_path_size - 1];
    reference_y_ = previous_path_y[previous_path_size - 1];
    x.push_back(reference_x_);
    y.push_back(reference_y_);

    reference_angle_ = atan2(reference_y_ - prev_reference_y, reference_x_ - prev_reference_x);
  }

  // Choose (two?) points ahead to draw a line. Thinking of a similar distance
  // increment, distance_increment * 25 and distance_increment * 50.
  std::vector<double> cartesian;
  cartesian = Helpers::getXY(current_s + 25, target_d, maps_s, maps_x, maps_y);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  cartesian = Helpers::getXY(current_s + 50, target_d, maps_s, maps_x, maps_y);
  x.push_back(cartesian[0]);
  y.push_back(cartesian[1]);

  // Now shift points to use the car's reference frame
  CartesianShift(x, y);

  // Now we use a spline (from http://kluge.in-chemnitz.de/opensource/spline/)
  // to plot a smooth trajectory that touches all our points.
  tk::spline s;
  s.set_points(x, y);  // Set must be sorted by x ascending!!

  double last_x = 0.0;

  // First we use up all points left over from previous path definition
  for (size_t i = 0; i < previous_path_size; ++i) {
    next_x.push_back(previous_path_x[i]);
    next_y.push_back(previous_path_y[i]);
  }

  // Then we create new points from the spline
  for (size_t i = 1; i <= path_points-previous_path_size; ++i) {
    double delta = distance_increment * i;
    // TODO: Improve next line, it seems wasteful.
    std::vector<double> calculated = CartesianUnshift(delta, s(delta));
    next_x.push_back(calculated[0]);
    next_y.push_back(calculated[1]);
  }

}

void PolynomialTrajectoryGenerator::CartesianShift(std::vector<double>& x,
                                                   std::vector<double>& y) {
  for (size_t i = 0; i < x.size(); ++i) {
    double shifted_x = x[i] - reference_x_;
    double shifted_y = y[i] - reference_y_;

    x[i] = shifted_x * cos(-reference_angle_)
        - shifted_y * sin(-reference_angle_);

    y[i] = shifted_x * sin(-reference_angle_)
        + shifted_y * cos(-reference_angle_);
  }
}

void PolynomialTrajectoryGenerator::CartesianUnshift(std::vector<double>& x,
                                                   std::vector<double>& y) {
  for (size_t i = 0; i < x.size(); ++i) {
    double rotated_x = x[i] * cos(reference_angle_)
        - y[i] * sin(reference_angle_);
    double rotated_y = x[i] * sin(reference_angle_)
        + y[i] * cos(reference_angle_);

    x[i] = rotated_x + reference_x_;

    y[i] = rotated_y + reference_y_;
  }
}

std::vector<double> PolynomialTrajectoryGenerator::CartesianUnshift(double x,
                                                                    double y) {
  double rotated_x = x * cos(reference_angle_)
      - y * sin(reference_angle_);
  double rotated_y = x * sin(reference_angle_)
      + y * cos(reference_angle_);

  /*
  std::vector<double> coordinates;
  coordinates.push_back(rotated_x + reference_x_);

  coordinates.push_back(rotated_y + reference_y_);
  return coordinates;
  */
  return {rotated_x + reference_x_, rotated_y + reference_y_};
}