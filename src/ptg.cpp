#include "ptg.h"

#include "spline.h"

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator() {}

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