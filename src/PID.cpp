#include "PID.h"

PID::PID(double Kp, double Kd, double Ki) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
}

double PID::update(double process_value) {

  double set_point = 0;

  double err = set_point - process_value;

  double err_derivative;
  if (has_prev_err) {
    err_derivative = err - prev_err;
  } else {
    err_derivative = 0;
    has_prev_err = true;
  }
  prev_err = err;

  err_integral += err;

  double control_variable =
    Kp * err +
    Kd * err_derivative +
    Ki * err_integral;

  // sse += pow(err, 2);
  // ++sse_count;

  return control_variable;
}
