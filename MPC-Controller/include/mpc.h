/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef MPC_MPC_H
#define MPC_MPC_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>


class MPC {
public:

  MPC(double latency, double max_speed);

  ~MPC() = default;

  //
  // Solve the model given an initial state and polynomial coefficients.
  //
  // @param state0: initial state of the car in [x, y, psi, v]
  // @param actuator0: initial actuator values in [steering, throttle]
  // @param ptsx: x coordinates (global coordinates) in the reference trajectory
  // @param ptsy: y coordinates (global coordinates) in the reference trajectory
  //
  bool solve(Eigen::VectorXd state0, Eigen::VectorXd actuator0,
             std::vector<double> ptsx, std::vector<double> ptsy);

  // latency getter
  double getLatency();

  // steering getter
  double getSteering();

  // throttle getter
  double getThrottle();

  // ref_x getter
  std::vector<double> getRefx();

  // ref_y getter
  std::vector<double> getRefy();

  // pred_x getter
  std::vector<double> getPredx();

  // pred_y getter
  std::vector<double> getPredy();

private:

  double lf_;  // distance between the front wheel and the vehicle center

  int order_;  // order of the polynomial fit
  Eigen::VectorXd ref_p_;  // polynomial coefficients for the reference trajectory
  std::vector<double> ref_x_;  // x coordinates of the reference trajectory
  std::vector<double> ref_y_;  // y coordinates of the reference trajectory

  std::vector<double> pred_x_;  // x coordinates of the predicted trajectory
  std::vector<double> pred_y_;  // y coordinates of the predicted trajectory

  double max_time_;  // maximum time of prediction in seconds
  double max_speed_;  // maximum speed in MPH

  // coefficients for predicting steering angle over time
  Eigen::VectorXd steering_coeff_;
  // coefficients for predicting throttle over time
  Eigen::VectorXd throttle_coeff_;

  double latency_;  // latency of the data acquisition system

  bool is_last_fit_success_;  // an indicator for the success of last fit

  //
  // update the points in the predicted trajectory
  //
  // @param state0: initial state of the car in [x, y, psi, v]
  // @param actuator: initial actuator values in [steering, throttle]
  //
  void updatePred(const Eigen::VectorXd& state0);

  //
  // update the polynomial coefficients of the reference trajectory as well as
  // the points in the reference trajectory
  //
  // @param x: x coordinates (global coordinates) in the reference trajectory
  // @param y: y coordinates (global coordinates) in the reference trajectory
  // @param x0: x coordinate of the origin of the new coordinate system in the old one
  // @param y0: y coordinate of the origin of the new coordinate system in the old one
  // @param psi: orientation of the new coordinate system with respect to the old one
  //
  void updateRef(const std::vector<double>& x, const std::vector<double>& y,
                 double x0, double y0, double psi);
};

#endif // MPC_MPC_H
