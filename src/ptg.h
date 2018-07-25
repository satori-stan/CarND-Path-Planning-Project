#ifndef PTG_H_
#define PTG_H_

#include <vector>

#include "helpers.h"

class PolynomialTrajectoryGenerator {

 public:
  static void StraightLine(
      const double current_x, const double current_y, const double current_yaw,
      std::vector<double>& new_x, std::vector<double>& new_y);
};

#endif  // PTG_H_