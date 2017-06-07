//
// Implementation of a model predictive controller
//
// Note: units in this project is not self-consistent
// We assume the speed is in MPH and the time is in second
//
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <uWS/uWS.h>

#include "json.hpp"
#include "mpc.h"
#include "utilities.h"


// for convenience
using json = nlohmann::json;


int main()
{
  uWS::Hub h;

  // Latency (in seconds)
  double latency = 0.1;

  // max speed
  double max_speed = 60;

  // mpc is initialized here!
  MPC mpc(latency, max_speed);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    // std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // global x/y positions of the waypoints
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];

          // current global x/y positions of the vehicle in meter
          double px = j[1]["x"];
          double py = j[1]["y"];

          // orientation of the vehicle in radians converted
          // from the Unity format to the standard format
          double psi = j[1]["psi"];

          // current velocity in MPH
          double v = j[1]["speed"];

          // current steering angle in Radians
          double steering = j[1]["steering_angle"];

          // current throttle between [-1, 1]
          double throttle = j[1]["throttle"];

          // search the optimized solution with given initial state
          Eigen::Vector4d state0 {px, py, psi, v};
          Eigen::Vector2d actuator0 {steering, throttle};
          mpc.solve(state0, actuator0, ptsx, ptsy);

          // the steering value should be normalized to [-1, 1] by dividing deg2rad(25)
          double steering_value = mpc.getSteering()/deg2rad(25);
          double throttle_value = mpc.getThrottle();

          json msgJson;

          msgJson["steering_angle"] = steering_value;
          msgJson["throttle"] = throttle_value;

          // the mpc predicted trajectory
          //
          // - Points are in reference to the vehicle's coordinate system
          // - The points in the simulator are connected by a green line
          msgJson["mpc_x"] = mpc.getPredx();
          msgJson["mpc_y"] = mpc.getPredy();

          // the reference trajectory
          //
          // - Points are in reference to the vehicle's coordinate system
          // - The points in the simulator are connected by a Yellow line
          msgJson["next_x"] = mpc.getRefx();
          msgJson["next_y"] = mpc.getRefy();

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          int latency_in_ms = mpc.getLatency()*1000;  // convert s to ms

          std::this_thread::sleep_for(std::chrono::milliseconds(latency_in_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
