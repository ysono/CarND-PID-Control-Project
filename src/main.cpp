#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <tuple>
#include "json.hpp"
#include "PID.h"
#include "pid_driver.cpp"
#include "twiddle_driver.cpp"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  Driver * driver = NULL;
  try {
    if (argc >= 2 && strcmp(argv[1], "twiddle") == 0) {
      std::cout << "using twiddle" << std::endl;
      double Kp = 0, Kd = 0, Ki = 0;
      if (argc == 5) {
        Kp = atof(argv[2]);
        Kd = atof(argv[3]);
        Ki = atof(argv[4]);
      }
      driver = new TwiddleDriver(Kp, Kd, Ki);
    } else if (argc == 5 && strcmp(argv[1], "pdi") == 0) {
      std::cout << "using user-defiend pid" << std::endl;
      double Kp = atof(argv[2]);
      double Kd = atof(argv[3]);
      double Ki = atof(argv[4]);
      driver = new PidDriver(Kp, Kd, Ki);
    } else if (argc == 2 && strcmp(argv[1], "help") == 0) {
      // noop
    } else if (argc == 2) {
      std::cout << "using pre-defined pid and user-defined speed" << std::endl;
      driver = new PidDriver(0.2, 2.8, 0.001, atof(argv[1]));
    } else if (argc == 1) {
      std::cout << "using pre-defined pid" << std::endl;
      driver = new PidDriver(0.2, 2.8, 0.001);
    }
  } catch(const std::exception & e) {
    // eg error in atof
    std::cerr << e.what() << std::endl;
  }
  if (driver == NULL) {
    std::cerr << "usage:" << std::endl
      << "  pid twiddle" << std::endl
      << "    tune with twiddle from scratch" << std::endl
      << "  pid twiddle <Kp> <Kd> <Ki>" << std::endl
      << "    tune with twiddle from specified hyperparameters" << std::endl
      << "  pid pdi <Kp> <Kd> <Ki>" << std::endl
      << "    drive with the specified hyperparameters for steering, and some default throttle" << std::endl
      << "  pid <speed>" << std::endl
      << "    drive with the final optimized hyperparameters for steering and throttle, and at a target speed in mph" << std::endl
      << "  pid" << std::endl
      << "    drive with the final optimized hyperparameters for steering and throttle, and at the default target speed of 40mph" << std::endl;
    exit(1);
  }

  h.onMessage([driver](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double steering, throttle;
          std::tie(steering, throttle) = driver->drive(cte, speed, angle);

          if (steering < -1) { steering = -1; }
          if (steering > 1) { steering = 1; }

          json msgJson;
          msgJson["steering_angle"] = steering;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &driver](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    delete driver;
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
