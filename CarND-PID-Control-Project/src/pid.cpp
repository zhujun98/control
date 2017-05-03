//
// pid.cpp
//

#include <iostream>
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

  d_time_ = 2;
  // Fill the queue first
  for (int i=0; i<d_time_; ++i)
  {
    p_error_history_.push(0.0);
  }
}

void PID::updateError(double cte)
{
  p_error_ = cte;

  integralError(); // update integral error

  derivativeError(); // update derivative error

}

void PID::derivativeError()
{
  d_error_ = (p_error_ - p_error_history_.front())/d_time_;
  p_error_history_.pop();
  p_error_history_.push(p_error_);
}

void PID::integralError()
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

double PID::calculate()
{
  double p_term = -k_p_*p_error_;
  double i_term = -k_i_*i_error_;
  double d_term = -k_d_*d_error_;

  double output = p_term + i_term + d_term;

  return output;
}


