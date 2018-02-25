#include "driver.h"

class PidDriver : public Driver {
private:

  double default_throttle = 0.3;

  PID steering_pid;
  PID * throttle_pid = NULL;

public:

  PidDriver(double Kp, double Kd, double Ki, double set_point_speed = 40) :
    steering_pid(Kp, Kd, Ki) {

    if (set_point_speed > 0) {
      throttle_pid = new PID(Kp, Kd, Ki, set_point_speed);
    }

    std::cout << "driving with pid hyperparams {" << Kp << " " << Kd << " " << Ki << "} and target speed " << set_point_speed << std::endl;
  }

  ~PidDriver() {
    delete throttle_pid;
  }

  std::tuple<double, double> drive(double cte, double speed, double angle) {
    return std::make_tuple(
      steering_pid.update(cte),
      (throttle_pid == NULL) ? default_throttle : throttle_pid->update(speed)
    );
  }
};
