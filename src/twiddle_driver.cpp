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

  /**
  * When using TwiddleDriver to tune, set throttle to a constant, to simplify things.
  */
  const double throttle_during_tuning = 0.3;
  const double throttle_during_resetting = 0.08;

  /**
  * During tuning phase, when the car goes out of bounds wrt this threshold, abort tuning and reset.
  */
  const double out_of_bounds_cte_thresh = 1.5;

  /**
  * During resetting phase, consider it done iff cte, speed, and angle reach
  * small values within their respective thresholds.
  *
  * However, during resetting, the angle measurement is often stuck at 0.4363,
  * so disable angle thresholding.
  */
  const double reset_cte_thresh = 0.2;
  const double reset_speed_thresh = 2.0;
  const double reset_angle_thresh = 999;

  /*
  * This set of hyperparameters is used for a pid controller to reset the car
  * back to the center at a near-stationary speed, after twiddle finishes evaluating
  * one set of hyperparameters, in preparation for evaluating the next set.
  *
  * These hyperparameters were inially guided by in-situ experiments with
  * `doc/initial-hyperparameters.py`, and were manually updated and recompiled
  * as better parameters were found. Now they're set to the final optimal values,
  * the same as those used by the default `PidDriver` in `main.cpp`.
  */
  const double K_for_resetting[3] = {0.2, 2.8, 0.001};
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
        std::cout << "abort evaluating " << twiddle.get_K_as_string() << " b/c out of bounds";
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

    // Here, phase is `resetting`.

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

    // Come to a near-stop, and use a preset PID plus user intervention to
    // move the car back to the center (and ideally pointing straight ahead,
    // but angle thresholding is disabled).

    double throttle_to_stop = (speed > reset_speed_thresh) ? -1 : throttle_during_resetting;

    if (fabs(cte) > out_of_bounds_cte_thresh) {
      // Note, simulator gives speed as a positive value even when reversing.
      return std::make_tuple(0, throttle_to_stop);
    }

    return std::make_tuple(pid_while_resetting.update(cte), throttle_to_stop);

  }
};
