//
// pid.cpp
//

#include <iostream>
#include <cmath>
#include "pid.h"


PID::PID() {}

PID::~PID() {}

void PID::init(double kp, double ki, double kd, int kd_interval)
{
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  sse_ = 0.0;

  kp_ = kp;
  ki_ = ki;
  kd_ = kd;

  i_time_ = 0;
  i_max_time_ = 100;

  d_time_ = kd_interval;

  // Fill the queue first
  for (int i=0; i<d_time_; ++i)
  {
    p_error_history_.push(0.0);
  }

  d_kp_ = 0.0;
  d_ki_ = 0.0;
  d_kd_ = 0.0;
  is_twiddle_ = false;
  is_twiddle_initialized_ = false;
  twiddle_count_ = 0;
  twiddle_duration_ = 1;
  twiddle_substate_ = 0;
  twiddle_state_ = 0;
}

double PID::calculate()
{
  double p_term = -kp_*p_error_;
  double i_term = -ki_*i_error_;
  double d_term = -kd_*d_error_;

  double output = p_term + i_term + d_term;

  return output;
}

void PID::updateError(double error)
{
  // update proportional error
  p_error_ = error;

  // update integral error
  if (p_error_*p_error_history_.front() < 0) {
    // reset integral error if the proportional error changes sign
    i_error_ = 0;
    i_time_ = 0;
  }
  else
  {
    // apply rolling average
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

  // update SSE
  sse_ += p_error_*p_error_;
}

void PID::twiddle_init(double d_kp, double d_ki, double d_kd, int t)
{
  sse_ = 0;
  sse_best_ = 1.0e6;

  d_kp_ = d_kp;
  d_ki_ = d_ki;
  d_kd_ = d_kd;
  twiddle_duration_ = t;

  is_twiddle_ = true;
  is_twiddle_initialized_ = false;
  twiddle_count_ = 0;
  twiddle_state_ = 0;
}

int PID::twiddle()
{
  double *k;
  double *d_k;

  // change one gain each time
  if (twiddle_state_ == 0)
  {
    k = &kp_;
    d_k = &d_kp_;
  }
  else if (twiddle_state_ == 1)
  {
    k = &ki_;
    d_k = &d_ki_;
  }
  else
  {
    k = &kd_;
    d_k = &d_kd_;
  }

  // at the beginning of each run
  if (twiddle_count_ == 0 && is_twiddle_initialized_)
  {
    sse_ = 0; // reset sse

    if (twiddle_substate_ == 0)
    {
      *k += *d_k; // try one direction
    }
    else
    {
      *k -= 2.0*(*d_k); // go to the other direction
    }
  }

  // at the end of each run
  if (twiddle_count_ >= twiddle_duration_)
  {
    twiddle_count_ = 0; // reset the twiddle count

    // output for debug
    std::cout << "SSE best: " << sse_best_ << "  "
              << "SSE: " << sse_ << "  "
              << "Kp: " << kp_ << "  "
              << "Ki: " << ki_ << "  "
              << "Kd: " << kd_ << std::endl;

    // the first finished run, initialize sse_best_
    if (!is_twiddle_initialized_) {
      sse_best_ = sse_;
      is_twiddle_initialized_ = true;
    }
    else
    {
      if (twiddle_substate_ == 0)
      {
        if (sse_ < sse_best_)
        {
          // there is improvement
          sse_best_ = sse_;
          *d_k *= 1.1;
          next_twiddle_state();
        }
        else
        {
          next_twiddle_substate();
        }
      }
      else
      {
        if (sse_ < sse_best_)
        {
          // there is improvement
          sse_best_ = sse_;
          *d_k *= 1.1;
        }
        else
        {
          *k += *d_k; // fail at both directions? go back
          *d_k *= 0.9; // decrease the step size
        }

        next_twiddle_substate();
        next_twiddle_state();
      }
    }

    return 1; // return 1 to reset the simulator
  }

  ++twiddle_count_;

  return 0;
}

double PID::get_sse()
{
  return sse_;
}

void PID::next_twiddle_substate()
{
  ++twiddle_substate_;

  if (twiddle_substate_ > 1) { twiddle_substate_ = 0; }
}

void PID::next_twiddle_state()
{
  ++twiddle_state_;

  if (twiddle_state_ > 2) { twiddle_state_ = 0; }
}


