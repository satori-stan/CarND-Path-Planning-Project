#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "path_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  PathPlanner planner(map_waypoints_x, map_waypoints_y,
      map_waypoints_s, 49.5, 10, 10);

#ifdef UWS_0_14_X
  h.onMessage([&planner, &map_waypoints_dx, &map_waypoints_dy](
      uWS::WebSocket<uWS::SERVER>* ws,
      char *data, size_t length, uWS::OpCode opCode) {
#else  // !UWS_0_14_X
  h.onMessage([&planner, &map_waypoints_dx, &map_waypoints_dy](
      uWS::WebSocket<uWS::SERVER> ws,
      char *data, size_t length, uWS::OpCode opCode) {
#endif  // !UWS_0_14_X

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            planner(j[1], next_x_vals, next_y_vals);

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));

#ifdef UWS_0_14_X
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else  // !UWS_0_14_X
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif  // !UWS_0_14_X

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";

#ifdef UWS_0_14_X
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else  // !UWS_0_14_X
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif  // !UWS_0_14_X

      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

#ifdef UWS_0_14_X
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
#else  // !UWS_0_14_X
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif  // !UWS_0_14_X
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef UWS_0_14_X
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
                         char *message, size_t length) {
    ws->close();
#else  // !UWS_0_14_X
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
#endif  // !UWS_0_14_X
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef UWS_0_14_X
  auto host = "127.0.0.1";
  if (h.listen(host, port)) {
#else  // !UWS_0_14_X
  if (h.listen(port)) {
#endif  // !UWS_0_14_X
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
