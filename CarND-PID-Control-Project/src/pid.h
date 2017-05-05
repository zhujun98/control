//
// pid.cpp
//

#ifndef PID_H
#define PID_H

#include <queue>


class PID
{
public:
  // constructor
  PID();

  // destructor
  virtual ~PID();

  //
  // Initiazation
  //
  // @param Kp: proportional term
  // @param Ki: integral term
  // @param Kd: derivative term
  //
  void init(double Kp, double Ki, double Kd);

  //
  // Calculate the output from the cross-track error
  //
  double calculate();

  //
  // sse getter
  //
  double get_sse();

  //
  // Update the PID error variables given cross track error.
  //
  // @param cte: cross-track error
  //
  void updateError(double error);


private:

  double p_error_; // error
  double i_error_; // integral of error
  double d_error_; // derivative of error
  double sse_; // sum of squared errors

  // pid coefficients
  double k_p_; // proportional gain
  double k_i_; // integral gain
  double k_d_; // derivative gain

  int i_time_; // current integral time
  int i_max_time_; // maximum integral time

  int d_time_; // derivative time interval

  // A queue to store the p error history
  // - Only for derivative error calculation!
  std::queue<double> p_error_history_;

};

#endif /* PID_H */
