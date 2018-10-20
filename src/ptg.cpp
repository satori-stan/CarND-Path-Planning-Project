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
  max_velocity_(max_speed * 1.609344 * 1000 / 3600),
  max_acceleration_(max_acceleration),
  max_jerk_(max_jerk),
  target_speed_(0.0),
  current_velocity_(0.0),
  current_acceleration_(0.0),
  current_normal_acceleration_(0.0),
  current_jerk_(0.0),
  previous_path_size_(0),
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
  cartesian = Helpers::getXY(current_s + 30, current_d, maps_s, maps_x, maps_y);
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

double LimitValue (double new_value, double limit_value) {
  return Helpers::sign(new_value) >= 0 ?
      min(new_value, limit_value) :
      max(new_value, -limit_value);
}

void PolynomialTrajectoryGenerator::FollowLaneAndLeadingCar(
    const double car_s,
    const double current_d,
    const double lane_start,
    const double lane_end,
    const json sensor_fusion,
    vector<Trajectory>& out) {

  double target_speed = max_speed_;
  // TODO: Turn into array of accelerations
  vector<SingleAxisTrajectoryBase> options{
    // First set is to keep velocity (no acceleration or jerk)
    {current_velocity_, 0.0, 0.0, 0.0},
    // Second set is unbound speed (but reach top speed "slowly")
    {max_velocity_, current_acceleration_, current_jerk_ + 0.5, 0.0},
    {max_velocity_, current_acceleration_ + 1.0, current_jerk_ + 0.5, 0.0},
    {max_velocity_, current_acceleration_ + 0.6, current_jerk_ + 0.5, 0.0},
    {max_velocity_, current_acceleration_ + 0.3, current_jerk_ + 0.5, 0.0},
    {max_velocity_, current_acceleration_ - 1.0, current_jerk_ + 0.5, 0.0},
    // Third set is following speed (allow for hard breaking)
    {current_velocity_ - 5.0, current_acceleration_ - 2.0, -1.0, 0.0}};

  // We calculate the acceleration and jerk based on a one second interval to
  // simplify the math.
  double time_delta = (kPathPoints - previous_path_size_) * kControllerExecutionTime;

  if (previous_path_size_ < kPathPoints) {
    // TODO: Sample paths with target velocities between the max and the current,
    //       assign costs and pick the one with the lowest cost.
    for (auto option = options.begin(); option != options.end(); ++option) {
      // Starting cost value
      out.emplace_back();
      Trajectory& trajectory = out.back();
      trajectory.cost = 0;

      double target_d = (lane_end + lane_start) / 2.0;

      vector<double> x(x_);
      vector<double> y(y_);
      // Choose (two?) points ahead to draw a line. Thinking of a similar distance
      // increment, distance_increment * 30 and distance_increment * 60.
      vector<double> cartesian;
      cartesian = Helpers::getXY(car_s + 30, target_d, maps_s_, maps_x_, maps_y_);
      //cartesian = Helpers::getXY((car_s + 30) % 6945.554, target_d, maps_s_, maps_x_, maps_y_);
      x.push_back(cartesian[0]);
      y.push_back(cartesian[1]);

      cartesian = Helpers::getXY(car_s + 60, target_d, maps_s_, maps_x_, maps_y_);
      //cartesian = Helpers::getXY((car_s + 60) % 6945.554, target_d, maps_s_, maps_x_, maps_y_);
      x.push_back(cartesian[0]);
      y.push_back(cartesian[1]);

      // Now shift points to use the car's reference frame
      CartesianShift(x, y);

      // Now we use a spline (from http://kluge.in-chemnitz.de/opensource/spline/)
      // to plot a smooth trajectory that touches all our points.
      tk::spline spline;
      spline.set_points(x, y);  // Set must be sorted by x ascending!!

      double delta = 0;
      double velocity = current_velocity_;
      trajectory.d_acceleration = current_normal_acceleration_ / 1.6180339;
      double acceleration = option->acceleration - abs(trajectory.d_acceleration);
      size_t i = 1;
      for (; i < (kPathPoints - previous_path_size_); ++i) {
        double elapsed = i * kControllerExecutionTime;
        velocity = (current_velocity_ + (acceleration * elapsed));
        delta = velocity * elapsed;
        //printf("%f, %f\n", velocity, acceleration);
        trajectory.d = spline(delta);
        // TODO: Improve next line, it seems wasteful.
        vector<double> calculated = CartesianUnshift(delta, trajectory.d);
        trajectory.x.push_back(calculated[0]);
        trajectory.y.push_back(calculated[1]);
      }
      trajectory.total_size = previous_path_size_ + i;
      trajectory.s = delta + car_s;
      trajectory.d = current_d - trajectory.d;
      trajectory.s_velocity = velocity;
      trajectory.s_acceleration = acceleration * 2.0;  // XXX: This is a hack, my acceleration calculations are off!
      trajectory.s_jerk = abs(trajectory.s_acceleration - current_acceleration_);  // Approximation since we use points for one second
      //trajectory.d_acceleration = 0.0;
      //trajectory.avg_d_acceleration = 0.0;
    }
  } else printf("%u\n", previous_path_size_);
}

void PolynomialTrajectoryGenerator::AssignBase(
    const double current_x,
    const double current_y,
    const double current_theta,
    const vector<double>& previous_x,
    const vector<double>& previous_y) {

  // Make sure we are building a path from a clean slate
  x_.clear();
  y_.clear();

  // Useful variables
  previous_path_size_ = previous_x.size();
  double prev_reference_x;
  double prev_reference_y;

  if (previous_path_size_ < 2) {
    reference_x_ = current_x;
    reference_y_ = current_y;
    reference_angle_ = current_theta;

    prev_reference_x = reference_x_ - cos(reference_angle_);
    prev_reference_y = reference_y_ - sin(reference_angle_);

    x_.push_back(prev_reference_x);
    y_.push_back(prev_reference_y);

    x_.push_back(reference_x_);
    y_.push_back(reference_y_);

  } else {
    prev_reference_x = previous_x[previous_path_size_ - 2];
    prev_reference_y = previous_y[previous_path_size_ - 2];
    x_.push_back(prev_reference_x);
    y_.push_back(prev_reference_y);

    reference_x_ = previous_x[previous_path_size_ - 1];
    reference_y_ = previous_y[previous_path_size_ - 1];
    x_.push_back(reference_x_);
    y_.push_back(reference_y_);

    reference_angle_ = atan2(reference_y_ - prev_reference_y,
                             reference_x_ - prev_reference_x);
    // For the excercise, use the last calculated speed as the current speed. In
    // a real setting, we would get this information from a sensor.
    /*
    current_velocity_ = sqrt(pow(reference_x_ - prev_reference_x, 2) +
                            pow(reference_y_ - prev_reference_y, 2)) /
                        kControllerExecutionTime;

    */
    //double previous_velocity = current_velocity_;
    if (previous_path_size_ > 2) {
      vector<double> velocities;
      size_t samples_for_a_second = static_cast<size_t>(1/kControllerExecutionTime) + 1;
      for (size_t i = 0; i < previous_path_size_ - 1; ++i) {
        double velocity = sqrt(pow(previous_x[i] - previous_x[i+1], 2) +
                              pow(previous_y[i] - previous_y[i+1], 2)) /
                          kControllerExecutionTime;
        velocities.push_back(velocity);
      }
      size_t samples = min(samples_for_a_second, velocities.size()) - 1;
      /*
      current_acceleration_ = 0.0;
      for (size_t i = 0; i < samples; ++i) {
        current_acceleration_ += velocities[i + 1] - velocities[i];
      }
      current_acceleration_ /= (samples * kControllerExecutionTime);
      */
      current_velocity_ = velocities.back();
      current_acceleration_ = (current_velocity_ - velocities.front()) / (samples * kControllerExecutionTime);
      /*
      current_jerk_ = accumulate(accelerations.begin(), accelerations.end(),
          0.0) / (accelerations.size() * kControllerExecutionTime);
      */
    }
  }
}

void PolynomialTrajectoryGenerator::ChangeLane(
    const double car_s,
    const double current_d,
    const double target_d,
    const json sensor_fusion,
    vector<Trajectory>& out) {

  vector<pair<double, double> > variations {
    {20, current_acceleration_},
    {30, current_acceleration_},
    {40, current_acceleration_},
    {20, 0.0},
    {30, 0.0},
    {40, 0.0},
    {20, current_acceleration_ + 0.6},
    {30, current_acceleration_ + 0.6},
    {40, current_acceleration_ + 0.6}};

  for (auto variation = variations.begin(); variation != variations.end(); ++variation) {
    vector<double> x(x_);
    vector<double> y(y_);
    // Choose (two?) points ahead to draw a line.
    std::vector<double> cartesian;
    cartesian = Helpers::getXY(car_s + variation->first, target_d, maps_s_, maps_x_, maps_y_);
    x.push_back(cartesian[0]);
    y.push_back(cartesian[1]);

    cartesian = Helpers::getXY(car_s + 60, target_d, maps_s_, maps_x_, maps_y_);
    x.push_back(cartesian[0]);
    y.push_back(cartesian[1]);

    // Now shift points to use the car's reference frame
    CartesianShift(x, y);

    // Now we use a spline (from http://kluge.in-chemnitz.de/opensource/spline/)
    // to plot a smooth trajectory that touches all our points.
    tk::spline spline;
    spline.set_points(x, y);  // Set must be sorted by x ascending!!

    // Allocate a new position in the output vector
    out.emplace_back();
    Trajectory& trajectory = out.back();

    double d_velocity_0 = 0.0;
    double lane_change_approximate_velocity =
        abs((target_d - current_d) / (variation->first / current_velocity_));  // 1.6180339;
    double change_timespan = variation->first / current_velocity_;
    double s_velocity = pow(current_velocity_, 2) - pow(lane_change_approximate_velocity, 2);
    trajectory.d_acceleration = (lane_change_approximate_velocity - d_velocity_0) / change_timespan;
    trajectory.s_acceleration = pow(variation->second, 2) - pow(abs(trajectory.d_acceleration), 2);
    double s = 0.0;
    double last_s = s;
    double last_d = 0.0;
    size_t i = 0;
    while (abs(current_d - target_d - trajectory.d) > (/*lane_width*/ 1.0) &&
        i < kPathPoints * 2) {
      double elapsed = ++i * kControllerExecutionTime;
      /*
      double d_velocity_1 = lane_change_approximate_velocity * exp(
        -pow(elapsed - (change_timespan / 2), 2) / (2 * pow(change_timespan / 6, 2))
      );
      */
      //d_velocity_0 = d_velocity_1;
      //trajectory.max_d_acceleration = max(abs(trajectory.d_acceleration), trajectory.max_d_acceleration);
      s_velocity = (current_velocity_ + (trajectory.s_acceleration * elapsed));
      s = s_velocity * elapsed;
      //printf("delta %f, tangential %f, normal %f\n", abs(current_d - target_d - d_delta), s_velocity, d_velocity_1);
      trajectory.d = spline(s);
      trajectory.d_velocity = (trajectory.d - last_d) / kControllerExecutionTime;
      trajectory.avg_d_acceleration = (trajectory.avg_d_acceleration * (i - 1) + (trajectory.d_velocity - d_velocity_0)) / i;
      d_velocity_0 = trajectory.d_velocity;
      last_d = trajectory.d;
      trajectory.s_velocity = (s - last_s) / kControllerExecutionTime;
      last_s = s;
      vector<double> calculated = CartesianUnshift(s, trajectory.d);
      trajectory.x.push_back(calculated[0]);
      trajectory.y.push_back(calculated[1]);
    }
    trajectory.total_size = previous_path_size_ + i;
    trajectory.s = car_s + s;
    trajectory.d = current_d - trajectory.d;
    //trajectory.s_velocity = s_velocity;
    trajectory.s_jerk = abs(trajectory.s_acceleration - current_acceleration_);  // Approximation since we use points for one second
    //printf("--\n");
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