//
// pid.cpp
//

#include <iostream>
#include <cmath>
#include "pid.h"


PID::PID() {}

PID::~PID() {}

void PID::init(double k_p, double k_i, double k_d)
{
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  k_p_ = k_p;
  k_i_ = k_i;
  k_d_ = k_d;

  i_time_ = 0;
  i_max_time_ = 100;

  d_time_ = 1;

  // Fill the queue first
  for (int i=0; i<d_time_; ++i)
  {
    p_error_history_.push(0.0);
  }
}

double PID::calculate()
{
  double p_term = -k_p_*p_error_;
  double i_term = -k_i_*i_error_;
  double d_term = -k_d_*d_error_;

  double output = p_term + i_term + d_term;

  return output;
}


void PID::updateError(double cte)
{
  // update proportional error
  p_error_ = cte;

  // update integral error (rolling average)
  if (p_error_*p_error_history_.front() < 0) {
    // reset integral error if the proportional error changes sign
    i_error_ = 0;
    i_time_ = 0;
  }
  else
  {
    if (i_time_ < i_max_time_)
    {
      i_error_ *= i_time_;
      i_error_ += p_error_;
      i_time_++;
      i_error_ /= i_time_;
    }
    else
    {
      i_error_ -= i_error_/i_max_time_;
      i_error_ += p_error_/i_max_time_;
    }
  }

  // update derivative error
  d_error_ = (p_error_ - p_error_history_.front())/d_time_;
  p_error_history_.pop();
  p_error_history_.push(p_error_);

  errorVariance(); // update sum of square error
}

void PID::errorVariance()
{
  if (v_time_ < v_max_time_)
  {
    variance_ *= v_time_;
    variance_ += p_error_*p_error_;
    v_time_++;
    variance_ /= v_time_;
  }
  else
  {
    variance_ -= variance_/v_max_time_;
    variance_ += p_error_*p_error_/v_max_time_;
  }
}

void PID::updateGain(double speed)
{
  double speed_ref = 33.0;
  double k_p_ref = 0.2;
  double k_i_ref = 0.2;
  double k_d_ref = 2.0;

  // The gains are both linearly dependent on the speed
  double ratio = speed/speed_ref;

//  k_p_ = k_p_ref*ratio;
//  k_i_ = k_i_ref*ratio;
  k_d_ = k_d_ref*(0.4*ratio*ratio + 0.6*ratio);
}
