#include <tuple>
#include "PID.h"

class Twiddle {
private:

  enum phase {
    testing_zero_hyperparams,
    testing_increased_K,
    testing_decreased_K,
    finalized_hyperparams
  };

  phase curr_phase = testing_zero_hyperparams;

  static const int num_hyperparams = 3;
  double K[num_hyperparams] = {0, 0, 0};
  double dK[num_hyperparams] = {1, 1, 1};
  short K_ind = -1;

  PID pid;

  const unsigned int num_iters_per_phase;

  double sse = 0; // summed squred error.
  int iter_count = 0;
  const double initial_mse = std::numeric_limits<double>::max();
  double lowest_mse = initial_mse;
  const double final_K_mse_thresh;

  const double final_K_sum_thresh;

  void change_phase(double mse) {
    if (curr_phase == testing_zero_hyperparams) {

      // We just finished measuring performance of {0,0,0}
      // Set baseline performance
      lowest_mse = mse;
      K_ind = 0;
      K[K_ind] += dK[K_ind];
      curr_phase = testing_increased_K;

    } else if (curr_phase == testing_increased_K) {

      if (mse < lowest_mse) {
        // Increasing the current K was beneficial
        // We'll keep the increased value for the current K and move on to the next K
        // Next time we increase/decrease the current K, we'll want to do it faster => make the current dK larger
        // Increase the next K
        // Keep the phase as "testing increased K"
        lowest_mse = mse;
        dK[K_ind] *= 1.1;
        K_ind = (K_ind + 1) % num_hyperparams;
        K[K_ind] += dK[K_ind];
      } else {
        // Increasing the current K was not beneficial
        // We'll try the current K again, but this time with a decreased value
        // Do not adjust the current dK. Adjust the current K to (the pre-increased K minus dK)
        // Change the phase to "testing decreased K"
        K[K_ind] -= 2 * dK[K_ind];
        curr_phase = testing_decreased_K;
      }

    } else if (curr_phase == testing_decreased_K) {

      if (mse < lowest_mse) {
        // Decreasing the current K was beneficial
        // We'll keep the decreased value for the current K
        // Next time we increase/decrease the current K, we'll want to do it faster => make the current dK larger
        lowest_mse = mse;
        dK[K_ind] *= 1.1;
      } else {
        // Neither increasing nor decreasing the current K was beneficial
        // We'll restore the original (pre-increase and pre-decrease) value for the current K
        // Next time we increase/decrease the current K, we'll want to do it slower => make the current dK smaller
        K[K_ind] += dK[K_ind];
        dK[K_ind] *= 0.9;
      }
      // Whether decreasing the curent K was beneficial or not ...
      // We're done with the current K. Move on to the next K
      // Increase the next K
      // Change the phase to "testing increased K"
      K_ind = (K_ind + 1) % num_hyperparams;
      K[K_ind] += dK[K_ind];
      curr_phase = testing_increased_K;

    }
  }

public:

  /*
  * num_iters_per_phase is the number of simulator events til ... TODO doc
  */
  Twiddle(
    unsigned int num_iters_per_phase_ = 200,
    double final_K_mse_thresh_ = 0.1,
    double final_K_sum_thresh_ = 0.2) :
      pid(K[0], K[1], K[2]),
      num_iters_per_phase(num_iters_per_phase_),
      final_K_mse_thresh(final_K_mse_thresh_),
      final_K_sum_thresh(final_K_sum_thresh_)
      {}

  /*
  * TODO doc the bool in output
  */
  std::tuple<double, bool> update(double process_value) {
    if (curr_phase == finalized_hyperparams) {
      return std::make_tuple(pid.update(process_value), false);
    }

    sse += pow(0 - process_value, 2);
    ++iter_count;

    bool is_end_of_phase = iter_count > num_iters_per_phase;
    if (is_end_of_phase) {
      // We've gone through enough iterations to evaluate the error from the current K

      double mse = sse / iter_count;

      if (mse < final_K_mse_thresh && K[0] + K[1] + K[2] < final_K_sum_thresh) {

        // Sufficiently good hyperparameters are discovered
        // Stop tuning and continue using the existing hyperparams with the existing derivative and integration state
        curr_phase = finalized_hyperparams;

      } else {

        change_phase(mse);

        sse = 0;
        iter_count = 0;

        pid = PID(K[0], K[1], K[2]);
      }
    }

    double control_value = pid.update(process_value);

    return std::make_tuple(control_value, is_end_of_phase);
  }

  void abort_current_K() {
    change_phase(initial_mse);
  }

  std::string get_K_as_string() {
    std::ostringstream os;
    os << K[0] << "," << K[1] << "," << K[2];
    return os.str();
  }
};
