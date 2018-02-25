#ifndef DRIVER_H
#define DRIVER_H

#include <tuple>

class Driver {
public:

  virtual ~Driver() = default;

  /**
  * Takes the current telemetry as args.
  *
  * Returns a tuple of (steering, throttle) control values.
  */
  virtual std::tuple<double, double> drive(double cte, double speed, double angle) = 0;

};

#endif /* DRIVER_H */
