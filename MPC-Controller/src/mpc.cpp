//
// Model predictive controller
//
// TODO: How to handle evil fit?
// TODO: Why the fit is prone to become evil when the PRED_TIME_STEP becomes small
//
#include <iostream>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "mpc.h"
#include "utilities.h"


namespace {
  // time step should be long enough to make the system be able to response timely
  const double PRED_TIME_STEP = 0.2;  // prediction time step in seconds
  const double TRACKING_TIME_STEP = 0.01;  // tracking time step in seconds
  const double MAX_STEERING = deg2rad(25);
  const double MAX_THROTTLE = 1.0;

  //
  // compute one actuator value after a certain time
  // @param dt: time step
  // @param p: coefficients used in calculation
  //
  template <class T, class Vector>
  T compute_actuator(Vector p, T t, T max_abs_x) {
    T result = p[0] + p[1]*t + p[2]*t*t + p[3]*t*t*t;

    if ( result > max_abs_x ) {
      result = max_abs_x;
    } else if ( result < - max_abs_x) {
      result = -max_abs_x;
    }

    return result;
  };

  using CppAD::AD;

  class FG_eval {
  public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    //
    // constructor
    //
    // @param state0: initial state [px, py, psi, v]
    // @param ref_p: coefficients of the polynomial fit for the reference trajectory
    // @param max_time: maximum time in one prediction (s)
    // @param max_speed: maximum speed of the vehicle
    // @param n_step: number of prediction steps
    // @param lf: distance between the front wheel and the vehicle center
    //
    FG_eval(Eigen::VectorXd state0, Eigen::VectorXd ref_p, double max_time,
            double max_speed, int n_step,double lf) {
      state0_ = state0;
      ref_p_ = ref_p;
      max_time_ = max_time;
      max_speed_ = max_speed;
      n_step_ = n_step;
      lf_ = lf;
    }

    //
    // Overload () operator.
    //
    // @param fg: objective and constraints
    //            The first one is objective and the rest are constraints.
    // @param x: variables
    //
    void operator()(ADvector& fg, const ADvector& x) {
      // initialize fg
      for (size_t i=0; i < fg.size(); ++i) { fg[i] = 0; }

      AD<double> px0 = state0_[0];
      AD<double> py0 = state0_[1];
      AD<double> psi = state0_[2];

      ADvector next_state(4);
      for (size_t i=0; i<next_state.size(); ++i) { next_state[i] = state0_[i]; }

      size_t steering_coeff_size = x.size()/2;
      size_t throttle_coeff_size = x.size()/2;

      ADvector steering_coeff(steering_coeff_size);
      for (size_t j=0; j<steering_coeff_size; ++j) {
        steering_coeff[j] = x[j];
      }
      ADvector throttle_coeff(throttle_coeff_size);
      for (size_t j=0; j<throttle_coeff.size(); ++j){
        throttle_coeff[j] = x[steering_coeff_size + j];
      }

      ADvector next_actuator(2);
      next_actuator[0] = steering_coeff[0];
      next_actuator[1] = steering_coeff[1];

      AD<double> cte = 0.0;  // current cross track error
      AD<double> coe = 0.0;  // current orientation error

      AD<double> ss_cte = 0.0;  // sum of square of cross track error
      AD<double> ss_coe = 0.0;  // sum of square of orientation error
      AD<double> ss_speed = 0.0;  // sum of square of speed

      AD<double> max_speed = 0.0;
      AD<double> min_speed = 200.0;
      AD<double> max_abs_cte = 0.0;
      AD<double> max_abs_coe = 0.0;

      AD<double> x_last = 0.0;
      AD<double> y_last = 0.0;

      AD<double> dt = max_time_/n_step_;
      AD<double> t = 0;
      for (int i=0; i<n_step_; ++i) {
        t += dt;

        // computer the next state
        // in each dt, the tracking is further divided into many sub-steps with
        // size TRACKING_TIME_STEP
        AD<double> sub_t = 0.0;
        while ( sub_t < dt - TRACKING_TIME_STEP ) {
          sub_t += TRACKING_TIME_STEP;
          next_state = globalKinematic(
              next_state, next_actuator, (AD<double>)TRACKING_TIME_STEP, lf_);
        }
        next_state = globalKinematic(next_state, next_actuator, dt - sub_t, lf_);

        // transform from global coordinates system to car's coordinate system
        ADvector X_new = globalToCar<ADvector, AD<double>>(
            next_state[0], next_state[1], px0, py0, psi);

        // calculate the cross track error`
        cte = X_new[1] - polyEval(ref_p_, X_new[0]);

        // calculate the orientation error
        if ( i > 0) {
          AD<double> dx = dt*next_state[3];
          AD<double> dy = polyEval(ref_p_, X_new[0] + dx/2) - polyEval(ref_p_, X_new[0] - dx/2);
          AD<double> ref_orientation = CppAD::atan(dy/dx);

          AD<double> current_orientation = CppAD::atan(
              (X_new[1] - y_last)/(X_new[0] - x_last));

          x_last = X_new[0];
          y_last = X_new[1];

          coe = current_orientation - ref_orientation;
        }

        // update several parameters

        if ( abs(cte) > max_abs_cte ) { max_abs_cte = abs(cte); }
        if ( abs(coe) > max_abs_coe ) { max_abs_coe = abs(coe); }
        if ( next_state[3] > max_speed ) { max_speed = next_state[3]; }
        if ( next_state[3] < min_speed ) { min_speed = next_state[3]; }

        // penalty functions

        ss_speed += next_state[3]*next_state[3];
        ss_cte += cte*cte;
        ss_coe += coe*coe;

        // use the coe in the second step to approximate the one in the first step
        if ( i == 1 ) { ss_coe += coe*coe; }

        // compute the next steering angle, we assume that the throttle is constant
        next_actuator[0] = compute_actuator(steering_coeff, t, (AD<double>)MAX_STEERING);
        next_actuator[1] = compute_actuator(throttle_coeff, t, (AD<double>)MAX_THROTTLE);
      }
      // objective function
      std::cout << "ss_cte: " << ss_cte << " "
                << "ss coe: " << ss_coe << " "
                << "ss speed: " << ss_speed << std::endl;

      fg[0] += 0.002*ss_cte + ss_coe;
      // speed control
      if ( state0_[3] < max_speed_ ) {
        fg[0] -= (max_speed_ - state0_[3])*1e-6*ss_speed;
      }

      // constraint functions
      fg[1] = max_abs_cte;

      return;
    }

  private:

    Eigen::VectorXd state0_;  // initial state of the optimization
    Eigen::VectorXd ref_p_;  // coefficients of the polynomial fit for the reference trajectory

    double max_time_;  // maximum time of prediction in seconds
    double max_speed_;  // maximum speed of the vehicle
    int n_step_;  // number of prediction steps

    AD<double> lf_;  // distance between the front wheel and the vehicle center
  };
}


// Implement MPC

MPC::MPC(double latency, double max_speed) {
  // This value was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  lf_ = 2.67;

  order_ = 3;
  ref_p_ = Eigen::VectorXd::Zero(order_+1);

  ref_x_ = std::vector<double>(20);
  ref_y_ = std::vector<double>(20);

  max_speed_ = max_speed;

  latency_ = latency;

  unsigned long n_step = 8;
  max_time_ = 8*PRED_TIME_STEP;

  pred_x_ = std::vector<double>(n_step);
  pred_y_ = std::vector<double>(n_step);

  steering_coeff_ = Eigen::VectorXd::Zero(4);
  throttle_coeff_ = Eigen::VectorXd::Zero(4);

  is_last_fit_success_ = true;
}

MPC::~MPC() {}

double MPC::getLatency() { return latency_; }

double MPC::getSteering() { return steering_coeff_[0]; }

double MPC::getThrottle() { return throttle_coeff_[0]; }

std::vector<double> MPC::getRefx() { return ref_x_; }

std::vector<double> MPC::getRefy() { return ref_y_; }

std::vector<double> MPC::getPredx() { return pred_x_; }

std::vector<double> MPC::getPredy() { return pred_y_; }

void MPC::updatePred(const Eigen::VectorXd& state0) {
  Eigen::VectorXd next_state(4);
  for (size_t i=0; i<next_state.size(); ++i) { next_state[i] = state0[i]; }

  Eigen::VectorXd next_actuator(2);
  next_actuator[0] = steering_coeff_[0];
  next_actuator[1] = throttle_coeff_[0];

  long n_step = pred_x_.size();
  double dt = max_time_/n_step;  // set time step
  double t = 0;
  for (int i=0; i<n_step; ++i) {
    t += dt;

    double sub_t = 0.0;
    while ( sub_t < dt - TRACKING_TIME_STEP ) {
      sub_t += TRACKING_TIME_STEP;
      next_state = globalKinematic(
          next_state, next_actuator, TRACKING_TIME_STEP, lf_);
    }
    next_state = globalKinematic(next_state, next_actuator, dt - sub_t, lf_);

    // transform from global coordinates system to car's coordinate system
    std::vector<double> X_new = globalToCar<std::vector<double>, double>(
        next_state[0], next_state[1], state0[0], state0[1], state0[2]);

    // assign reference trajectory for each step
    pred_x_[i] = X_new[0];
    pred_y_[i] = X_new[1];

    // computer the actuator in the next state
    next_actuator[0] = compute_actuator(steering_coeff_, t, MAX_STEERING);
    next_actuator[1] = compute_actuator(throttle_coeff_, t, MAX_THROTTLE);
  }

}

void MPC::updateRef(const std::vector<double>& x, const std::vector<double>& y,
                    double px0, double py0, double psi) {
  assert(x.size() == y.size());

  size_t length0 = x.size();

  Eigen::VectorXd ref_x(length0);
  Eigen::VectorXd ref_y(length0);

  // 1. Fit the read out reference trajectory

  for (size_t i=0; i<x.size(); ++i) {
    // transform from global coordinates system to car's coordinate system
    std::vector<double> X_new = globalToCar<std::vector<double>, double>(
        x[i], y[i], px0, py0, psi);
    // store the reference trajectory in the car's coordinate system
    ref_x[i] = X_new[0];
    ref_y[i] = X_new[1];
  }

  // polynomial fit of the reference trajectory
  ref_p_ = leastSquareFit(ref_x, ref_y, order_);

  // 2. Generate a finer reference trajectory

  double x_max = ref_x[ref_x.size() - 1];
  long ref_length = ref_x_.size();
  double x_step = x_max / ref_length;

  for (int i=0; i<ref_length; ++i) {
    ref_x_[i] = x_step*i;
    ref_y_[i] = polyEval(ref_p_, ref_x_[i]);
  }
}

bool MPC::solve(Eigen::VectorXd state0, Eigen::VectorXd actuator0,
                std::vector<double> ptsx, std::vector<double> ptsy)
{
  //
  // update the reference trajectory
  // the reference trajectory should be calculated using the measured state
  // vector since the way points and vehicle state were measured at the same
  // time prior to the current time.
  //

  updateRef(ptsx, ptsy, state0[0], state0[1], state0[2]);

  //
  // estimate the current status to compensate the latency
  //
  Eigen::VectorXd estimated_state0(4);
  for (int i=0; i<state0.size(); ++i) { estimated_state0[i] = state0[i]; }

  double sub_t = 0.0;
  while ( sub_t < latency_ - TRACKING_TIME_STEP ) {
    sub_t += TRACKING_TIME_STEP;
    estimated_state0 = globalKinematic(
        estimated_state0, actuator0, TRACKING_TIME_STEP, lf_);
  }
  estimated_state0 = globalKinematic(estimated_state0, actuator0, latency_ - sub_t, lf_);

  //
  // set up the optimizer
  //

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // number of variables:
  size_t nx = steering_coeff_.size() + throttle_coeff_.size();

  Dvector xi(nx), xl(nx), xu(nx);
  // initialize variables
  if ( is_last_fit_success_ ) {
    for (int i=0; i<steering_coeff_.size(); ++i) {
      xi[i] = steering_coeff_[i];
    }
    for (int i=0; i<throttle_coeff_.size(); ++i) {
      xi[steering_coeff_.size()+i] = throttle_coeff_[i];
    }
  } else {
    for (int i=0; i<nx; ++i) { xi[i] = 0; }
    xi[0] = actuator0[0];
    xi[steering_coeff_.size()] = actuator0[1];
  }

  // set variable boundaries

  xl[0] = -MAX_STEERING; xu[0] = MAX_STEERING;
  xl[1] = -0.1; xu[1] = 0.1;
  xl[2] = -0.3; xu[2] = 0.3;
  xl[3] = -0.3; xu[3] = 0.3;

  xl[4] = 0; xu[4] = MAX_THROTTLE;
  xl[5] = -0.2; xu[5] = 0.2;
  xl[6] = 0; xu[6] = 0;
  xl[7] = 0; xu[7] = 0;

  // number of constraints:
  size_t ng = 1;

  Dvector gl(ng), gu(ng);

  // set constraint boundaries
  // It is easy to get an evil fit with multiple constraints

  // constraint 1: maximum absolute cte
  gl[0] = 0.0;
  gu[0] = 2.0;

  // object that computes objective and constraints
  FG_eval fg_eval(estimated_state0, ref_p_, max_time_, max_speed_, (int)pred_x_.size(), lf_);

  // options for IPOPT solver
  std::string options;

  options += "Integer print_level  0\n";
  // Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // structure that holds the solution of the optimization
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

  is_last_fit_success_ = true;
  is_last_fit_success_ &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Check some of the solution values
  //      possible values for the result status
  //      enum status_type {
  //           0: not_defined,
  //           1: success,
  //           2: maxiter_exceeded,
  //           3: stop_at_tiny_step,
  //           4: stop_at_acceptable_point,
  //           5: local_infeasibility,
  //           6: user_requested_stop,
  //           7: feasible_point_found,
  //           8: diverging_iterates,
  //           9: restoration_failure,
  //           10: error_in_step_computation,
  //           11: invalid_number_detected,
  //           12: too_few_degrees_of_freedom,
  //           13: internal_error,
  //           14: unknown
  //      };

  // Print out the results
  std::cout << "solution status: " << solution.status << std::endl;
  std::cout << "Cost " << solution.obj_value << std::endl;
  std::cout << "optimized variables: ";
  for (int i=0; i<solution.x.size(); ++i) {
    std::cout << solution.x[i] << "  ";
  }
  std::cout << std::endl;

  std::cout << "constraints: ";
  for (int i=0; i<solution.g.size(); ++i) {
    std::cout << solution.g[i] << "  ";
  }
  std::cout << std::endl;

  // assign the optimized values
  for (int i=0; i<steering_coeff_.size(); ++i) {
    steering_coeff_[i] = solution.x[i];
  }
  for (int i=0; i<throttle_coeff_.size(); ++i) {
    throttle_coeff_[i] = solution.x[steering_coeff_.size()+i];
  }

  // update the predicted trajectory
  updatePred(estimated_state0);

  return is_last_fit_success_;
}
