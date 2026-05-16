// ============================================================================
//  auv_control/ts_fuzzy.hpp
//
//  T-S (Takagi and Sugeno) fuzzy state-feedback controller exactly as
//  described in Section 2 of Zhang et al., Sensors 24, 3029.
//
//      x(t)  = [ u v w q r ]^T        (5-dim body-frame twist, surge/sway/heave
//                                      + pitch rate + yaw rate)
//      u(t)  = sum_j mu_j(theta) * K_j * (x(t) - x_ref(t))
//
//  Six rules on the 2D premise grid theta1 = u, theta2 = r:
//      theta1 in {0.5, 1.0}          (Eq. 18)
//      theta2 in {-0.1, 0.0, 0.1}    (Eq. 19)
//
//  Each rule has its own state-feedback gain K_j (4x5). Since the paper
//  does not publish numerical K_j values, we synthesise them from the
//  closed-loop response requirements (pole placement / simple PD on the
//  actuated directions).
// ============================================================================
#ifndef AUV_CONTROL__TS_FUZZY_HPP_
#define AUV_CONTROL__TS_FUZZY_HPP_

#include <array>
#include <Eigen/Dense>

#include "auv_control/auv_params.hpp"

namespace auv_control {

using Gain = Eigen::Matrix<double, 4, 5>;   // K_j  (wrench-shaped output)

class TSFuzzyController {
 public:
  TSFuzzyController();

  // Compute sum_j mu_j(theta) * K_j * (x - x_ref)  — Eq. (13) of the paper.
  //   x       : current state [u v w q r]^T
  //   x_ref   : desired state [u_ref v_ref w_ref q_ref r_ref]^T
  //   returns : desired body-frame wrench (4-dim: Fx, Fz, My, Mz).
  WrenchVec compute(const StateVec & x, const StateVec & x_ref) const;

  // Update fault factors so the controller can adapt its behavior.
  void set_fault_factors(const std::array<double, kNumThrusters> & f);

  // Set the control sampling period (for integral term).
  void set_dt(double dt) { dt_ = dt; }

  // Last set of membership weights mu_j (for logging / inspection).
  const std::array<double, 6> & last_weights() const { return mu_; }

  // Clear the accumulated error integral to prevent windup during transitions.
  void reset_integral() const { error_integral_.setZero(); }

 private:
  // ---- Membership functions (Eqs. 18, 19) ----
  //   M_theta1=0.5(t) = (2 + sin(theta1))/5
  //   M_theta1=1.0(t) = (3 - sin(theta1))/5
  //   M_theta2=-0.1   = (cos(theta2) + 2)/5
  //   M_theta2= 0.0   = (sin(theta2) + 3)/5
  //   M_theta2= 0.1   = (-sin(theta2) - cos(theta2) + 5)/5
  static double M_t1_low (double th1);
  static double M_t1_high(double th1);
  static double M_t2_neg (double th2);
  static double M_t2_zero(double th2);
  static double M_t2_pos (double th2);

  // Six operating-point gain matrices K_1 .. K_6 (Eqs. 20-25).
  std::array<Gain, 6> K_;

  // Integral gains (shared across rules for simplicity).
  Gain Ki_;

  // Cached membership weights mu_1..mu_6 from the last compute() call.
  mutable std::array<double, 6> mu_{};

  // Fault factors [T1..T6], where 1.0=healthy, 0.0=total failure.
  std::array<double, kNumThrusters> fault_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};

  // State error integral for the PI-like fuzzy law.
  mutable StateVec error_integral_ = StateVec::Zero();
  double dt_ = 0.02;   // default 50Hz
};

}  // namespace auv_control

#endif  // AUV_CONTROL__TS_FUZZY_HPP_
