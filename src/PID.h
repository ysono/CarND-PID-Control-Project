#ifndef PID_H
#define PID_H

class PID {
private:

  double Kp, Kd, Ki;

  bool has_prev_err = false;

  double prev_err;

  double err_integral = 0;

public:

  PID(double Kp, double Kd, double Ki);

  /*
  * Update the PID error variables given cross track error.
  *
  * Setpoint is 0.
  */
  double update(double process_value);

};

#endif /* PID_H */
