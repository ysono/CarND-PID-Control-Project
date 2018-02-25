#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <tuple>
#include "json.hpp"
#include "PID.h"
#include "twiddle.cpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

PID create_default_pid() {
  return PID(0.2, 4.0, 0.004); // TODO cli arg?
}

int main()
{
  uWS::Hub h;

  // double default_Kp = 0.2, default_Kd = 3.0, default_Ki = 0.004;

  // PID default_pid(0.2, 4.0, 0.004); // TODO cli arg?
  // PID pid(8.751, 20.192, 0.0295);
  PID default_pid = create_default_pid();

  Twiddle twiddle;

  bool is_tuning = true;
  const double reset_cte_thresh = 0.2;
  const double reset_angle_thresh = 5.0 / 180.0 * pi();
  const double out_of_bounds_cte_thresh = 1.8;

  h.onMessage(
    [&default_pid, &twiddle, &is_tuning, &reset_cte_thresh, &reset_angle_thresh, &out_of_bounds_cte_thresh]
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

          // double steer_value;
          // // if (counter % 30 > 15) {
          // if (cte < 0) {
          //   steer_value = M_PI/4;
          // } else {
          //   steer_value = -M_PI/4;
          // }

          // double steer_value = default_pid.update(cte); // TODO restore this. Add cli option.

          double steer_value;

          if (is_tuning) {
            if (fabs(cte) > out_of_bounds_cte_thresh) {
              std::cout << "abort tuning " << twiddle.get_K_as_string() << " due to excessive error" << std::endl;
              twiddle.abort_tuning();
              is_tuning = false;
              default_pid = create_default_pid();
              steer_value = default_pid.update(cte);
            } else {
              std::string K_str = twiddle.get_K_as_string();
              bool did_tuning_finish;
              std::tie(steer_value, did_tuning_finish) = twiddle.update(cte);
              if (did_tuning_finish) {
                is_tuning = false;
                std::cout << "successfully ended tuning " << K_str << std::endl;
              }
            }
          } else {
            if (fabs(cte) < reset_cte_thresh && fabs(angle) < reset_angle_thresh) {
              std::cout << "start tuning " << twiddle.get_K_as_string() << std::endl;
              is_tuning = true;
              bool _;
              std::tie(steer_value, _) = twiddle.update(cte);
            } else {
              steer_value = default_pid.update(cte);
            }
          }

          // if (is_in_between_tuning) {
          //   if (fabs(cte) < reset_cte_thresh && fabs(angle) < reset_angle_thresh) {
          //     is_in_between_tuning = false;
          //     bool _;
          //     std::tie(steer_value, _) = twiddle.update(cte);
          //     std::cout << "start tuning " << twiddle.get_K_as_string() << std::endl;
          //   } else {
          //     steer_value = default_pid.update(cte);
          //   }
          // } else if (fabs(cte) > out_of_bounds_cte_thresh) {
          //   std::cout << "aborted tuning " << twiddle.get_K_as_string() << " due to excessive error" << std::endl;
          //   twiddle.abort_tuning();
          //   is_in_between_tuning = true;
          //   steer_value = default_pid.update(cte);
          // } else {
          //   bool is_tuning_done;
          //   std::tie(steer_value, is_tuning_done) = twiddle.update(cte);
          //   if (is_tuning_done) {
          //     is_in_between_tuning = true;
          //     std::cout << "successfully ended the last twiddle phase" << std::endl;
          //   }
          // }

          


          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // std::cout
          //   << "CTE: " << cte
          //   << " angle: " << angle
          //   << " Steering Value: " << steer_value
          //   << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
