#ifndef TWIDDLE
#define TWIDDLE
#define NUM_K 3
#endif

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

  double K[NUM_K];
  double dK[NUM_K];

  short K_ind = -1;

  PID pid;

  const unsigned int num_iters_per_phase = 2400;

  double sse = 0; // summed squred error.
  int iter_count = 0;
  const double initial_mse = std::numeric_limits<double>::max();
  double lowest_mse = initial_mse;
  const double final_K_mse_thresh = 0.001;

  const double final_K_sum_thresh = 0.5;

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
        // Keep the increased value for the current K and move on to the next K
        // Next time we increase/decrease the current K, we'll want to do it faster => make the current dK larger
        // Increase the next K
        // Keep the phase as "testing increased K"
        lowest_mse = mse;
        dK[K_ind] *= 1.1;
        K_ind = (K_ind + 1) % NUM_K;
        K[K_ind] += dK[K_ind];
      } else {
        // Increasing the current K was not beneficial
        // We'll try the current K again, but this time with a decreased value, as long as this decreased value is positive

        // Decreased current K is `the pre-increased K minus dK`
        double decreased_current_K = K[K_ind] - 2 * dK[K_ind];

        if (decreased_current_K > 0) {
          // Adopt the decreased current K
          // Do not adjust the current dK.
          // Change the phase to "testing decreased K"
          K[K_ind] = decreased_current_K;
          curr_phase = testing_decreased_K;
        } else {
          // Skip the decreased K
          // Restore the current K to its pre-increase value
          // Next time we increase/decrease the current K, we'll want to do it slower => make the current dK smaller
          // Move on to the next K
          // Increment the next K
          // Keep the phase as "testing increased K"
          K[K_ind] -= dK[K_ind];
          dK[K_ind] *= 0.9;
          K_ind = (K_ind + 1) % NUM_K;
          K[K_ind] += dK[K_ind];
        }
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
        // Restore the original (pre-increase and pre-decrease) value for the current K
        // Next time we increase/decrease the current K, we'll want to do it slower => make the current dK smaller
        K[K_ind] += dK[K_ind];
        dK[K_ind] *= 0.9;
      }
      // Whether decreasing the curent K was beneficial or not ...
      // We're done with the current K. Move on to the next K
      // Increase the next K
      // Change the phase to "testing increased K"
      K_ind = (K_ind + 1) % NUM_K;
      K[K_ind] += dK[K_ind];
      curr_phase = testing_increased_K;

    }
  }

public:

  /**
  * TODO document how zeros are handled. eg why 1,1,1?
  */
  Twiddle(double Kp, double Kd, double Ki) :
    K{(Kp == 0) ? 1 : Kp, (Kd == 0) ? 1 : Kd, (Ki == 0) ? 1 : Ki},
    dK{K[0], K[1], K[2]},
    pid(K[0], K[1], K[2])
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
      std::cout << "mse is " << mse << std::endl;

      if (mse < final_K_mse_thresh || K[0] + K[1] + K[2] < final_K_sum_thresh) {

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

    sse = 0;
    iter_count = 0;

    pid = PID(K[0], K[1], K[2]);
  }

  std::string get_K_as_string() {
    std::ostringstream os;
    os << "{" << K[0] << " " << K[1] << " " << K[2] << "}";
    return os.str();
  }
};
