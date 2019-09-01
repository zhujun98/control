/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef MPC_UTILITIES_H
#define MPC_UTILITIES_H

#include <iostream>
#include <cmath>
#include <Eigen3.3/Eigen/Dense>


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


//
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//
// @param s: input string
//
inline std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("}]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }

  return "";
}

//
// the global kinematic model.
//
// @param state: state of the car in [x, y, psi, v]
// @param actuator: actuator values in [steering, throttle]
// @param dt: time step (s)
// @param lf: distance between the front wheel and the vehicle center
//
// @return next_state: state of the car after time dt
//
template <class Vector, class T>
inline Vector globalKinematic(const Vector& state, const Vector& actuator, T dt, T lf) {
  Vector next_state(state.size());

  next_state[0] = state[0] + state[3]*cos(state[2])*dt;
  next_state[1] = state[1] + state[3]*sin(state[2])*dt;
  next_state[2] = state[2] - state[3]/lf*actuator[0]*dt;
  next_state[3] = state[3] + actuator[1]*dt;

  return next_state;
}


//
// evaluate a polynomial function
// a[0] + a[1]x + a[2]x^2 + ... + a[n-1]x^(n-1)
//
// @param a: coefficients of the polynomial
// @param x: variable
//
// @return result: evaluation of the polynomial function
//
template <class T>
inline T polyEval(Eigen::VectorXd a, T x) {
  T result = 0.0;

  for (int i = 0; i < a.size(); ++i) {
    result += a[i] * pow(x, i);
  }

  return result;
}


//
// solve a least square problem
//
// @param xvals: vector of x variables
// @param yvals: vector of y variables
// @param order: order of the polynomial fit
//
inline Eigen::VectorXd leastSquareFit(const Eigen::VectorXd& xvals,
                                      const Eigen::VectorXd& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  // assign the first column
  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  // assign the rest columns
  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i+1) = A(j, i) * xvals[j];
    }
  }

  // solve the least square problem using QR decomposition
  // Eigen::VectorXd result = A.householderQr().solve(yvals);  // fastest
  Eigen::VectorXd result = A.colPivHouseholderQr().solve(yvals);
  // Eigen::VectorXd result = A.fullPivHouseholderQr().solve(yvals);  // stablest

  return result;
}

//
// transform coordinate from one coordinate system to the other
//
// @param X: coordinates (x, y) in the old coordinate system to be transformed
// @param X0: origin (x0, y0) of the new coordinate system in the old one
// @param psi: orientation of the new coordinate system with respect to the old one
//
template <class Vector, class T>
inline Vector globalToCar(T px, T py, T px0, T py0, T psi) {
  Vector X_new(2);

  T c = cos(psi);
  T s = sin(psi);
  T dx = px - px0;
  T dy = py - py0;

  // a rotation clock-wise about the origin
  X_new[0] = dx*c + dy*s;
  X_new[1] = -dx*s + dy*c;

  return X_new;
}

#endif // MPC_UTILITIES_H
