#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <tuple>
#include "json.hpp"
#include "PID.h"
#include "twiddle_manager.cpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

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

// PID create_default_pid() {
//   // return PID(0.2, 4.0, 0.004); // TODO cli arg?

//   return PID(8.751, 20.192, 0.0295);
// }

int main()
{
  uWS::Hub h;

  // double default_Kp = 0.2, default_Kd = 3.0, default_Ki = 0.004;

  // PID default_pid(0.2, 4.0, 0.004); // TODO cli arg?
  // PID pid(8.751, 20.192, 0.0295);
  // PID default_pid = create_default_pid();

  // Twiddle twiddle;

  TwiddleManager twiddle_manager;

  // bool is_tuning = true;
  // // TODO in readme say how these were chosen
  // const double reset_cte_thresh = 0.2;
  // const double reset_angle_thresh = 5.0 / 180.0 * M_PI;
  // const double out_of_bounds_cte_thresh = 1.5;

  h.onMessage(
    // [&default_pid, &twiddle, &is_tuning, &reset_cte_thresh, &reset_angle_thresh, &out_of_bounds_cte_thresh]
    [&twiddle_manager]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          // double steering = default_pid.update(cte); // TODO restore this. Add cli option.

          double steering, throttle;
          std::tie(steering, throttle) = twiddle_manager.get_control(cte, speed, angle);

          // double steering;
          // double throttle = 0.3;

          // if (is_tuning) {

          //   bool is_done_with_prev_K = false;
          //   if (fabs(cte) > out_of_bounds_cte_thresh) {
          //     std::cout << "abort " << twiddle.get_K_as_string() << " due to excessive error";
          //     twiddle.abort_current_K();
          //     is_done_with_prev_K = true;
          //   } else {
          //     std::tie(steering, is_done_with_prev_K) = twiddle.update(cte);
          //     if (is_done_with_prev_K) {
          //       std::cout << "successfully evaluated the previous K";
          //     }
          //   }

          //   if (is_done_with_prev_K) {
          //     std::cout << "; next K to evaluate is " << twiddle.get_K_as_string() << std::endl;
          //     is_tuning = false;
          //     default_pid = create_default_pid();
          //     steering = default_pid.update(cte);
          //     throttle = 0.08;
          //   }

          // } else {
          //   if (fabs(cte) < reset_cte_thresh && fabs(angle) < reset_angle_thresh) {
          //     std::cout << "start evaluating " << twiddle.get_K_as_string() << std::endl;
          //     is_tuning = true;
          //     bool _;
          //     std::tie(steering, _) = twiddle.update(cte);
          //   } else {
          //     steering = default_pid.update(cte);
          //     throttle = 0.08;
          //   }
          // }

          if (steering < -1) { steering = -1; }
          if (steering > 1) { steering = 1; }

          // std::cout
          //   << "CTE: " << cte
          //   << " angle: " << angle
          //   << " Steering Value: " << steering
          //   << std::endl;

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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
