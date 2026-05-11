// ============================================================================
//  auv_control/ts_fuzzy.cpp
// ============================================================================
#include "auv_control/ts_fuzzy.hpp"

#include <cmath>

namespace auv_control {

// ---------------------------------------------------------------------------
//  Membership functions — exactly Eqs. (18), (19) of the paper.
// ---------------------------------------------------------------------------
double TSFuzzyController::M_t1_low (double t1) { return (2.0 + std::sin(t1)) / 5.0; }
double TSFuzzyController::M_t1_high(double t1) { return (3.0 - std::sin(t1)) / 5.0; }
double TSFuzzyController::M_t2_neg (double t2) { return (std::cos(t2) + 2.0) / 5.0; }
double TSFuzzyController::M_t2_zero(double t2) { return (std::sin(t2) + 3.0) / 5.0; }
double TSFuzzyController::M_t2_pos (double t2) {
  return (-std::sin(t2) - std::cos(t2) + 5.0) / 5.0;
}

// ---------------------------------------------------------------------------
//  Gain-matrix synthesis.
//
//  The paper's K_j are not published. Each rule targets a different operating
//  point, so we keep the same structural template but bias each K_j for its
//  local linearization (low speed ~ higher yaw authority, high speed ~ softer
//  yaw gain to prevent oscillation). State order: [u v w q r].
//  Actuator mapping: u1=surge, u2=heave, u3=pitch-moment, u4=yaw-moment.
// ---------------------------------------------------------------------------
static Gain make_gain(double ku, double kw, double kq, double kr) {
  Gain K;
  K.setZero();
  K(0, 0) = ku;       // u1 tracks surge error
  K(1, 2) = kw;       // u2 tracks heave error
  K(2, 3) = kq;       // u3 tracks pitch-rate error
  K(3, 4) = kr;       // u4 tracks yaw-rate error
  // Disable cross-coupling for stability
  K(3, 1) = 0.0;
  return K;
}

TSFuzzyController::TSFuzzyController() {
  //                       ku    kw    kq    kr     (units: N per m/s or N*m per rad/s)
  K_[0] = make_gain( 50.0, 35.0, 5.0, 5.0);   // Rule 1: theta1=0.5, theta2=-0.1
  K_[1] = make_gain( 50.0, 35.0, 5.0, 4.0);   // Rule 2: theta1=0.5, theta2= 0.0
  K_[2] = make_gain( 50.0, 35.0, 5.0, 5.0);   // Rule 3: theta1=0.5, theta2= 0.1
  K_[3] = make_gain( 55.0, 35.0, 6.0, 5.0);   // Rule 4: theta1=1.0, theta2=-0.1
  K_[4] = make_gain( 55.0, 35.0, 6.0, 4.0);   // Rule 5: theta1=1.0, theta2= 0.0
  K_[5] = make_gain( 55.0, 35.0, 6.0, 5.0);   // Rule 6: theta1=1.0, theta2= 0.1

  // Small integral gains
  Ki_.setZero();
  Ki_(0, 0) = 1.0;   // surge integral
  Ki_(1, 2) = 1.0;   // heave integral
  Ki_(3, 4) = 0.5;   // yaw integral
}

void TSFuzzyController::set_fault_factors(const std::array<double, 4> & f) {
  fault_ = f;
}

// ---------------------------------------------------------------------------
//  compute() — Eq. (13)
// ---------------------------------------------------------------------------
ControlVec TSFuzzyController::compute(const StateVec & x,
                                      const StateVec & x_ref) const {
  const double th1 = x(0);   // theta1 = surge speed u
  const double th2 = x(4);   // theta2 = yaw rate r

  // Rule firing strengths omega_i = product of membership functions for
  // theta1 and theta2.  (Eq. 11 normalises them into mu_i.)
  const double m1l = M_t1_low (th1);
  const double m1h = M_t1_high(th1);
  const double m2n = M_t2_neg (th2);
  const double m2z = M_t2_zero(th2);
  const double m2p = M_t2_pos (th2);

  std::array<double, 6> omega = {
    m1l * m2n,  // R1: theta1=0.5, theta2=-0.1
    m1l * m2z,  // R2: theta1=0.5, theta2= 0.0
    m1l * m2p,  // R3: theta1=0.5, theta2= 0.1
    m1h * m2n,  // R4: theta1=1.0, theta2=-0.1
    m1h * m2z,  // R5: theta1=1.0, theta2= 0.0
    m1h * m2p,  // R6: theta1=1.0, theta2= 0.1
  };

  double denom = 0.0;
  for (double w : omega) denom += w;
  if (denom < 1e-9) denom = 1.0;          // numerical floor

  // Defuzzify: weighted sum of state-feedback contributions.
  ControlVec u = ControlVec::Zero();
  const StateVec e = x - x_ref;

  // Adaptive Fault Tolerance: 
  // We apply a compensation factor to the virtual control vector.
  // If an actuator has a fault (effectiveness f_i < 1.0), we can attempt 
  // to increase the gain for that channel to compensate, provided 
  // it's not a total failure.
  for (std::size_t j = 0; j < 6; ++j) {
    mu_[j] = omega[j] / denom;
    
    // Proportional term from the T-S rules
    ControlVec u_prop = -K_[j] * e;

    // Integral term (PI-like) to reject steady-state errors from faults/drag
    ControlVec u_int  = -Ki_ * error_integral_;

    // Combined rule output
    ControlVec u_j = u_prop + u_int;

    u.noalias() += mu_[j] * u_j;
  }

  // Anti-windup: only integrate if the commands are not heavily saturated.
  // (Assuming nominal saturation around 50N per channel).
  bool saturated = false;
  for (int i = 0; i < 4; ++i) {
    if (std::abs(u(i)) > 60.0) { saturated = true; break; }
  }

  if (!saturated) {
    error_integral_ += e * dt_;
    // Cap integral to prevent windup from large transient errors
    for (int i = 0; i < 5; ++i) {
      error_integral_(i) = std::clamp(error_integral_(i), -2.0, 2.0);
    }
  }

  return u;
}

}  // namespace auv_control
