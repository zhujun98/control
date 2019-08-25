/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef PID_H
#define PID_H

#include <deque>


class PID
{
public:
  PID() = default;

  ~PID() = default;

  bool is_twiddle_; // twiddle flag

  //
  // Initiazation
  //
  // @param k_p: proportional term
  // @param k_i: integral term
  // @param k_d: derivative term
  // @param dt: rolling average time span for p_error_
  //            and derivative time interval for d_error_
  //
  void init(double k_p, double k_i, double k_d, int dt);

  //
  // Update the PID error variables given cross track error.
  //
  // @param error: cross-track error
  //
  void updateError(double error);

  //
  // Calculate the output from the cross-track error
  //
  double calculate();

  //
  // run twiddle optimization
  //
  int twiddle();

  //
  // initialize twiddle
  //
  // @param d_kp: initial step size for kp
  // @param d_ki: initial step size for ki
  // @param d_kd: initial step size for kd
  // @param t: time duration for each run (here the number of steps)
  //
  void twiddle_init(double d_kp, double d_ki, double d_kd, int t);

  //
  // sse getter
  //
  double get_sse();


private:

  double p_error_; // error
  double i_error_; // integral of error
  double d_error_; // derivative of error
  double sse_; // sum of squared errors

  double kp_; // proportional gain
  double ki_; // integral gain
  double kd_; // derivative gain

  int i_time_; // current integral time
  int i_max_time_; // maximum integral time

  int d_time_; // derivative time interval

  // A queue to store the p error history
  // - Only for derivative error calculation!
  std::deque<double> error_history_;

  // parameters for twiddle
  bool is_twiddle_initialized_; // twiddle initialization flag
  int twiddle_count_; // time step count for twiddle
  int twiddle_duration_; // max duration for twiddle
  int twiddle_substate_; // sub-state for twiddle: 0 (increase) or 1 (decrease)
  int twiddle_state_; // state for twiddle: 0 (p), 1 (i) and 2 (d)
  double sse_best_; // best sse achieved
  double d_kp_; // current kp step
  double d_ki_; // current ki step
  double d_kd_; // current kd step

  //
  // shift to the next twiddle state
  //
  void next_twiddle_state();

  //
  // shift to the next twiddle substate
  //
  void next_twiddle_substate();

};

#endif /* PID_H */
