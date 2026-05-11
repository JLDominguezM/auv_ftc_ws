// ============================================================================
//  auv_control/thrust_allocator.hpp
//
//  Weighted pseudo-inverse thrust allocator (Section 3.2, Eq. 37) and
//  active-set quadratic-programming re-allocator (Section 4.2, Eqs. 42-49).
//
//  Given:
//      B          : 5x4 configuration matrix (tau = B * u)
//      tau_des    : desired body-frame wrench (5-dim)
//      W          : 4x4 diagonal priority matrix, w_i = exp(1/f_i - 1)
//                    f_i = fault factor in (0, 1],   f_i == 0 ==> w_i = +inf
//      u_min,u_max: actuator box constraints (scalar, symmetric)
//
//  Algorithm:
//      1) u_unconstrained = W^{-1} B^T (B W^{-1} B^T)^{-1} tau_des      (Eq. 37)
//      2) If min_i u_i >= u_min AND max_i u_i <= u_max:   return u.
//      3) Otherwise solve the constrained QP:
//             min (1/2)||v||^2
//             s.t.  B v  = tau_des
//                   u_min <= v <= u_max
//         by an active-set method on the simple-bound constraints.
// ============================================================================
#ifndef AUV_CONTROL__THRUST_ALLOCATOR_HPP_
#define AUV_CONTROL__THRUST_ALLOCATOR_HPP_

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

#include "auv_control/auv_params.hpp"

namespace auv_control {

class ThrustAllocator {
 public:
  explicit ThrustAllocator(const AllocParams & geom);

  // Set the current fault factor f_i ∈ [0, 1] for each of the 4 actuator
  // channels. f_i = 1 is healthy, 0 is total loss. The priority weights
  // are derived per Eq. (32):  w_i = exp(1/f_i - 1) for f_i > 0,
  // and w_i = 1e12 (saturated) for f_i = 0.
  void set_fault_factors(const std::array<double, 4> & f);

  // Core allocation entry point. Returns the 4-dim actuator command u,
  // best matching tau_des while respecting [u_min, u_max] box limits.
  // status_out is set to:
  //    0 = unconstrained pseudo-inverse solution used
  //    1 = saturation hit, QP re-allocation used
  //    2 = QP could not reach tau_des exactly (constraints active, best
  //        feasible returned).
  ControlVec allocate(const WrenchVec & tau_des,
                      double u_min, double u_max,
                      int * status_out = nullptr) const;

  // Actual wrench produced by a fault-effective command u_applied.
  // Accounts for actuator-effectiveness reduction:  tau_actual = B * diag(f) * u.
  WrenchVec actual_wrench(const ControlVec & u_cmd) const;

  const Eigen::Matrix<double, 4, 4> & B() const { return B_; }

  // Last-computed diagnostic info.
  std::string last_report() const { return report_; }

 private:
  // Weighted pseudo-inverse core (Eq. 37, no constraint handling).
  ControlVec pseudo_inverse(const WrenchVec & tau) const;

  // Active-set QP on box-constrained least squares:
  //   min (1/2) v^T v   s.t.  B v = tau_des,  u_min <= v <= u_max.
  ControlVec solve_qp(const WrenchVec & tau_des,
                      double u_min, double u_max,
                      bool * converged) const;

  AllocParams                geom_;
  Eigen::Matrix<double, 4, 4> B_;
  Eigen::Matrix<double, 4, 4> W_;     // priority matrix (diagonal)
  std::array<double, 4>      fault_;  // f_1..f_4 in [0, 1]
  mutable std::string        report_;
};

}  // namespace auv_control

#endif  // AUV_CONTROL__THRUST_ALLOCATOR_HPP_
