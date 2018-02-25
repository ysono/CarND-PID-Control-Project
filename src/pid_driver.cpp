#include "driver.h"

class PidDriver : public Driver {
private:

  double steering = 0.3; // TODO

  PID pid;

public:

  PidDriver(double Kp, double Kd, double Ki) :
    pid(Kp, Kd, Ki)
    {}

  std::tuple<double, double> drive(double cte, double speed, double angle) {
    return std::make_tuple(pid.update(cte), steering);
  }
};
