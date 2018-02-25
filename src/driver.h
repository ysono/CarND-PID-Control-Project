#ifndef DRIVER_H
#define DRIVER_H

#include <tuple> // TODO

class Driver {
public:

  virtual ~Driver() = default;

  /**
  * TODO doc
  */
  virtual std::tuple<double, double> drive(double cte, double speed, double angle) { return std::make_tuple(0, 0); } //  = 0; // TODO

};

#endif /* DRIVER_H */
