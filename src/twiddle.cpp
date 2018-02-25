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
    testing_increased_term,
    testing_decreased_term,
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
      curr_phase = testing_increased_term;

    } else if (curr_phase == testing_increased_term) {

      if (mse < lowest_mse) {
        // Increasing the current term was beneficial
        // Keep the increased value for the current term and move on to the next term
        // Next time we increase/decrease the current term, we'll want to do it faster => make the delta-current-term larger
        // Increase the next term
        // Keep the phase as "testing increased term"
        lowest_mse = mse;
        dK[K_ind] *= 1.1;
        K_ind = (K_ind + 1) % NUM_K;
        K[K_ind] += dK[K_ind];
      } else {
        // Increasing the current term was not beneficial
        // We'll try the current term again, but this time with a decreased value, as long as this decreased value is positive
        // (Even if we initialize `dK` to be the same as `K`, which the contructor indeed does,
        // negative term is still possible. The stochastic nature of the simulation means that,
        // if previous decrease of this term improved mse, twiddle "thinks" that
        // further decreasing the same term may further improve mse.)

        // Decreased current term is `the pre-increased term minus delta-current-term`
        double decreased_current_K = K[K_ind] - 2 * dK[K_ind];

        if (decreased_current_K > 0) {
          // Adopt the decreased current term
          // Do not adjust the current dK.
          // Change the phase to "testing decreased term"
          K[K_ind] = decreased_current_K;
          curr_phase = testing_decreased_term;
        } else {
          // Skip the decreased term
          // Restore the current term to its pre-increase value
          // Next time we increase/decrease the current term, we'll want to do it slower => make the delta-current-term smaller
          // Move on to the next term
          // Increment the next term
          // Keep the phase as "testing increased term"
          K[K_ind] -= dK[K_ind];
          dK[K_ind] *= 0.9;
          K_ind = (K_ind + 1) % NUM_K;
          K[K_ind] += dK[K_ind];
        }
      }

    } else if (curr_phase == testing_decreased_term) {

      if (mse < lowest_mse) {
        // Decreasing the current term was beneficial
        // We'll keep the decreased value for the current term
        // Next time we increase/decrease the current term, we'll want to do it faster => make the delta-current-term larger
        lowest_mse = mse;
        dK[K_ind] *= 1.1;
      } else {
        // Neither increasing nor decreasing the current term was beneficial
        // Restore the original (pre-increase and pre-decrease) value for the current term
        // Next time we increase/decrease the current term, we'll want to do it slower => make the delta-current-term smaller
        K[K_ind] += dK[K_ind];
        dK[K_ind] *= 0.9;
      }
      // Whether decreasing the curent term was beneficial or not ...
      // We're done with the current term. Move on to the next term
      // Increase the next term
      // Change the phase to "testing increased term"
      K_ind = (K_ind + 1) % NUM_K;
      K[K_ind] += dK[K_ind];
      curr_phase = testing_increased_term;

    }

    sse = 0;
    iter_count = 0;

    pid = PID(K[0], K[1], K[2]);
  }

public:

  /**
  * This twiddle implementation skips tuning any parameter that is zero or negative.
  * Therefore if any provided hyperparameter is zero, set it arbitrarily to 1.
  */
  Twiddle(double Kp, double Kd, double Ki) :
    K{(Kp == 0) ? 1 : Kp, (Kd == 0) ? 1 : Kd, (Ki == 0) ? 1 : Ki},
    dK{K[0], K[1], K[2]},
    pid(K[0], K[1], K[2])
    {}

  /**
  * Returns tuple of
  * (
  *   steering control value,
  *   whether evaluation of the current set of hyperparameters is done
  * )
  *
  * The latter bool notifies the consumer code to prepare the simulation for
  * the next round of evaluation.
  */
  std::tuple<double, bool> update(double process_value) {
    if (curr_phase == finalized_hyperparams) {
      return std::make_tuple(pid.update(process_value), false);
    }

    sse += pow(0 - process_value, 2); // steering is not added to error, but perhaps it should be.
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
    os << "{" << K[0] << " " << K[1] << " " << K[2] << "}";
    return os.str();
  }
};
