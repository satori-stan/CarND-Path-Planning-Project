#include "ptg.h"

#include <iostream>

#include "spline.h"

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator() :
    // TODO: Set starting speed to zero once a speedup policy has been set-up
    target_speed_(49.5) {}

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
  // Previous path's end s and d values 
  double end_path_s = sensor_data["end_path_s"];
  double end_path_d = sensor_data["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = sensor_data["sensor_fusion"];

  std::vector<double> frenet = Helpers::getFrenet(car_x, car_y,
                                                  car_yaw, maps_x, maps_y);
  double current_s = frenet[0];
  // Should use frenet[1], but the sparsity of the map data can cause the value
  // to vary. Instead we express the value of d in terms of the lane we are
  // keeping.
  double lane_width = 4;  // In m
  int current_lane = 2;
  double lane_start = lane_width * (current_lane - 1);
  double current_d = (current_lane - 0.5) * lane_width;
  double lane_end = lane_width * current_lane;

  // Make sure we slow down if there is a car in our path
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
    if (adversary_d > lane_start && adversary_d < lane_end) {
      // The other car is in our lane!
      // TODO: Beware, the s value wraps around
      // TODO: Make vehicle safety distance into a variable
      if (adversary_s < end_path_s && (end_path_s-adversary_s) < 30) {
        // We are too close!
        // TODO: Make increments and decrements proportional to the acceleration
        //       limits.
        // TODO: Make the car speed up if the way ahead is clear (up to the
        //       speed limit).
        target_speed_ *= 0.99;
        std::cout << target_speed_ << std::endl;
      }
    }
  }

  // 50 points into the future, but only move in s coordinate
  size_t path_points = 50;
  // mph * km_per_mile * meters_per_km / seconds_per_hour
  double target_velocity = target_speed_ * 1.609344 * 1000 / 3600;  // In m/s
  double controller_execution_time = 0.02;  // In s
  double distance_increment = controller_execution_time * target_velocity;  // path_points;

  // We build a path buffer to make sure the trajectory is always smooth
  std::vector<double> x;
  std::vector<double> y;
  size_t previous_path_size = previous_path_x.size();
  double first_x;
  // TODO: Use more points from the previous path to account for lag between
  //       path calculation and actual position upon execution.
  if (previous_path_size < 2) {
    // If we have less than two points left over from our last path calculation,
    // we approximate two points with the current position and 1 unit in the
    // past based on the current yaw.
    first_x = car_x - cos(Helpers::deg2rad(car_yaw));
    x.push_back(first_x);
    y.push_back(car_y - sin(Helpers::deg2rad(car_yaw)));

    x.push_back(car_x);
    y.push_back(car_y);
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
    double x = car_x + delta;
    next_x.push_back(x);
    next_y.push_back(s(x));
  }
}