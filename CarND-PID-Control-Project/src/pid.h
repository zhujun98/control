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
  // Update the PID error variables given cross track error.
  //
  // @param cte: cross-track error
  //
  void updateError(double cte);

  //
  // Update the PID coefficients
  //
  // @param speed: measured car speed
  //
  void updateGain(double speed);

private:

  double p_error_; // error
  double i_error_; // integral of error
  double d_error_; // derivative of error
  double variance_; // variance of error

  // pid coefficients
  double k_p_; // proportional gain
  double k_i_; // integral gain
  double k_d_; // derivative gain

  int i_time_; // current integral time
  int i_max_time_; // maximum integral time

  int d_time_; // derivative time interval

  int v_time_; // current error variance integral time
  int v_max_time_; // maximum integral time for error variance

  std::queue<double> p_error_history_; // A queue to store the p error history

  //
  // Update the error variance
  //
  void errorVariance();
};

#endif /* PID_H */
