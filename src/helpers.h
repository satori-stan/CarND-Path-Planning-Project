#ifndef HELPERS_H_
#define HELPERS_H_

#ifdef WIN32
#  define _USE_MATH_DEFINES
#endif  // WIN32

#include <math.h>
#include <vector>

using namespace std;

class Helpers {

 public:
  // For converting back and forth between radians and degrees.
  static constexpr double pi() { return M_PI; }
  static inline double deg2rad(double x) { return x * pi() / 180; }
  static inline double rad2deg(double x) { return x * 180 / pi(); }
  template <typename T> static inline int sign (T value) {
    return (T(0) < value) - (value < T(0));
  }

  static double distance(double x1, double y1, double x2, double y2);

  static int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  static vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  // Transform from Frenet s,d coordinates to Cartesian x,y
  static vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

  static int UnsignedToSigned(unsigned x);

  static double Logistic(double x);

};

#endif  // HELPERS_H_