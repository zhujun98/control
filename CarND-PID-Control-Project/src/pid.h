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
  void init(double Kp, double Ki, double Kd);

  //
  // Calculate the output from the cross-track error
  //
  double calculate();

  //
  //
  // Update the PID error variables given cross track error.
  //
  // @param cte: cross-track error
  //
  void updateError(double cte);

private:

  double p_error_;
  double i_error_;
  double d_error_;

  // pid coefficients
  double k_p_; // proportional term
  double k_i_; // integral term
  double k_d_; // derivative term

  int i_time_; // current integral time
  int i_max_time_; // maximum integral time

  int d_time_; // derivative time interval
  std::queue<double> p_error_history_; // A queue to store the p error history

  //
  // update the integral error
  //
  void integralError();

  //
  // Update the derivative error
  //
  void derivativeError();
};

#endif /* PID_H */
