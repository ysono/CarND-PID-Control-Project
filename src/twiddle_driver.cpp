#include <tuple>
#include "driver.h"
#include "twiddle.cpp"

class TwiddleDriver : public Driver {
private:

  enum phase {
    resetting,
    tuning
  };

  phase curr_phase = resetting;

  Twiddle twiddle;

  const double throttle_during_tuning = 0.3;
  const double throttle_during_resetting = 0.08;

  // TODO in readme say how these were chosen
  const double out_of_bounds_cte_thresh = 1.5;
  const double reset_cte_thresh = 0.2;
  const double reset_speed_thresh = 2.0;
  const double reset_angle_thresh = 999; // When resetting, angle measurement is often stuck at 0.4363, so disable this threshold.

  // TODO explain these numbers
  const double K_for_resetting[3] = {0.2, 2.8, 0.001}; // TODO put 3 in a .h somewhere
  PID pid_while_resetting;

  const int cout_width = 80;
  const std::string backspaces = std::string(cout_width, '\b');

public:

  TwiddleDriver(double Kp, double Kd, double Ki) :
    twiddle(Kp, Kd, Ki),
    pid_while_resetting(K_for_resetting[0], K_for_resetting[1], K_for_resetting[2])
  {
    std::cout << "initial K to evaluate is " << twiddle.get_K_as_string() << std::endl;
  }

  std::tuple<double, double> drive(double cte, double speed, double angle) {

    if (curr_phase == tuning) {

      if (fabs(cte) > out_of_bounds_cte_thresh) {
        std::cout << "abort evaluating " << twiddle.get_K_as_string() << " b/c out of bounds"; // TODO replace log
        twiddle.abort_current_K();
      } else {
        double steering;
        bool will_change_K;
        std::tie(steering, will_change_K) = twiddle.update(cte);
        if (will_change_K) {
          std::cout << "successfully evaluated the previous K"; // cannot print K b/c it's already been updated
        } else {
          return std::make_tuple(steering, throttle_during_tuning);
        }
      }

      // Phase is `resetting`.

      std::cout << "; next K to evaluate will be " << twiddle.get_K_as_string() << std::endl;
      curr_phase = resetting;
      pid_while_resetting = PID(K_for_resetting[0], K_for_resetting[1], K_for_resetting[2]);

    }

    // Phase is `resetting`.

    bool is_ready_to_tune_again =
      fabs(cte) < reset_cte_thresh && fabs(speed) < reset_speed_thresh && fabs(angle) < reset_angle_thresh;

    std::ostringstream states_os;
    states_os << "Reset. CTE: " << cte << " speed: " << speed << " angle: " << angle;
    if (is_ready_to_tune_again) { states_os << " READY"; }
    std::cout << std::left << std::setw(cout_width) << states_os.str() << backspaces;

    if (is_ready_to_tune_again) {
      std::ostringstream info_os;
      info_os << "start evaluating " << twiddle.get_K_as_string();
      std::cout << std::left << std::setw(cout_width) << info_os.str() << std::endl;

      curr_phase = tuning;
      double steering;
      bool _;
      std::tie(steering, _) = twiddle.update(cte);
      return std::make_tuple(steering, throttle_during_tuning);
    }

    // Come to a near-stop, and except user to move the car back to the center pointing straight ahead.

    double throttle_to_stop = (speed > reset_speed_thresh) ? -1 : throttle_during_resetting; // TODO use pid?

    if (fabs(cte) > out_of_bounds_cte_thresh) {
      // Note, simulator gives speed as a positive value even when reversing.
      return std::make_tuple(0, throttle_to_stop);
    }

    return std::make_tuple(pid_while_resetting.update(cte), throttle_to_stop);

  }
};


/*
          double steering;
          double throttle = 0.3;

          if (is_tuning) {

            bool is_done_with_prev_K = false;
            if (fabs(cte) > out_of_bounds_cte_thresh) {
              std::cout << "abort " << twiddle.get_K_as_string() << " due to excessive error";
              twiddle.abort_current_K();
              is_done_with_prev_K = true;
            } else {
              std::tie(steering, is_done_with_prev_K) = twiddle.update(cte);
              if (is_done_with_prev_K) {
                std::cout << "successfully evaluated the previous K";
              }
            }

            if (is_done_with_prev_K) {
              std::cout << "; next K to evaluate is " << twiddle.get_K_as_string() << std::endl;
              is_tuning = false;
              default_pid = create_default_pid();
              steering = default_pid.update(cte);
              throttle = 0.08;
            }

          } else {
            if (fabs(cte) < reset_cte_thresh && fabs(angle) < reset_angle_thresh) {
              std::cout << "start evaluating " << twiddle.get_K_as_string() << std::endl;
              is_tuning = true;
              bool _;
              std::tie(steering, _) = twiddle.update(cte);
            } else {
              steering = default_pid.update(cte);
              throttle = 0.08;
            }
          }
*/
