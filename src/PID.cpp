#include "PID.h"

PID::PID(double Kp, double Kd, double Ki, double set_point) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
  this->set_point = set_point;
}

double PID::update(double process_value) {

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

  return control_variable;
}
