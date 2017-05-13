#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include "json.hpp"
#include "pid.h"
#include "utilities.h"


// for convenience
using json = nlohmann::json;

int main()
{
  uWS::Hub h;

  std::ofstream historyFile;

  // steer pid controller
  PID steer_pid;
  steer_pid.init(0.075, 0.075, 2.50, 2);
//  steer_pid.twiddle_init(0.01, 0.01, 0.02, 1500);

  // throttle pid controller
  PID throttle_pid;
  throttle_pid.init(1.0, 0.0, 4.0, 2);

  h.onMessage([&steer_pid, &throttle_pid, &historyFile](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // "4" signifies a websocket message
    // "2" signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          int is_restart = 0;

          //
          // update steer
          //
          steer_pid.updateError(cte);
          double steer_sse = steer_pid.get_sse();
          double steer_value = steer_pid.calculate(); // Calculate steering value [-1, 1]
          // cap the steering
          if (steer_value < -1.0) { steer_value = -1.0; }
          if (steer_value > 1.0) { steer_value = 1.0; }

          if (steer_pid.is_twiddle_) { is_restart = steer_pid.twiddle(); }

          //
          // update throttle
          //
          double throttle_input = -1.0/(4*cte*cte + 1.0);
          throttle_pid.updateError(throttle_input);
          double throttle_value = throttle_pid.calculate();

          if (throttle_pid.is_twiddle_) { is_restart = throttle_pid.twiddle(); }

          // reset the simulator
          if (is_restart)
          {
            historyFile.close();

            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

            sleep(5); // give time for the simulator to restart
          }

          // write the results to file for visualization later
          historyFile << cte << "\t" << steer_value << "\t" << steer_sse
                      << "\t" << speed << "\n";

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // printout message when not twiddling
          if ( !steer_pid.is_twiddle_ && !throttle_pid.is_twiddle_ ) {
            std::cout << msg << std::endl;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &historyFile](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
    historyFile.open("./output/history.txt");
  });

  h.onDisconnection([&h, &historyFile](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    historyFile.close();
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
