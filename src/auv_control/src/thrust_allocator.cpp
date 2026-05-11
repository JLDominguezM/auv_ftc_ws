// ============================================================================
//  auv_control/thrust_allocator.cpp
// ============================================================================
#include "auv_control/thrust_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace auv_control {

namespace {
constexpr double kWeightSaturation = 1.0e12;  // stand-in for w_i = +inf
constexpr double kEpsKKT           = 1.0e-6;
constexpr int    kMaxQPIter        = 40;
}  // namespace

// ---------------------------------------------------------------------------
ThrustAllocator::ThrustAllocator(const AllocParams & geom) : geom_(geom) {
  B_ = build_B(geom);
  W_ = Eigen::Matrix<double, 4, 4>::Identity();
  fault_ = {1.0, 1.0, 1.0, 1.0};
}

void ThrustAllocator::set_fault_factors(const std::array<double, 4> & f) {
  fault_ = f;
  W_.setZero();
  for (int i = 0; i < 4; ++i) {
    const double fi = std::clamp(f[i], 0.0, 1.0);
    double wi;
    if (fi <= 1.0e-6) {
      wi = kWeightSaturation;
    } else {
      // Eq. (32):  w_i = exp(1/f_i - 1). Healthy -> 1, severe -> very large.
      wi = std::exp(1.0 / fi - 1.0);
    }
    W_(i, i) = wi;
  }
}

// ---------------------------------------------------------------------------
//  Weighted pseudo-inverse (Eq. 37, updated for fault effectiveness F):
//        B_eff = B * diag(f)
//        u = W^{-1} B_eff^T (B_eff W^{-1} B_eff^T)^{-1} tau
// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::pseudo_inverse(const WrenchVec & tau) const {
  Eigen::Matrix<double, 4, 4> Beff = B_;
  for (int j = 0; j < 4; ++j) Beff.col(j) *= fault_[j];

  const Eigen::Matrix<double, 4, 4> Winv = W_.inverse();
  const Eigen::Matrix<double, 4, 4> M    = Beff * Winv * Beff.transpose();
  
  // Use LDLT for the 4x4 square system.
  const Eigen::Matrix<double, 4, 4> M_reg = M + 1e-6 * Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Matrix<double, 4, 1> lambda = M_reg.ldlt().solve(tau);
  return Winv * Beff.transpose() * lambda;
}

// ---------------------------------------------------------------------------
//  Active-set QP:   min (1/2)||v||^2
//                   s.t.  B_eff v  = tau_des
//                         u_min <= v <= u_max
// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::solve_qp(const WrenchVec & tau_des,
                                     double u_min, double u_max,
                                     bool * converged) const {
  if (converged) *converged = false;

  Eigen::Matrix<double, 4, 4> Beff = B_;
  for (int j = 0; j < 4; ++j) Beff.col(j) *= fault_[j];

  // Start from the minimum-norm point of the effective system.
  const Eigen::Matrix<double, 4, 4> M0 = Beff * Beff.transpose() + 1e-6 * Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Matrix<double, 4, 1> lambda = M0.ldlt().solve(tau_des);
  ControlVec v = Beff.transpose() * lambda;

  // Active bounds tracker: 0 = free, -1 = pinned at u_min, +1 = pinned at u_max.
  std::array<int, 4> pin = {0, 0, 0, 0};

  for (int iter = 0; iter < kMaxQPIter; ++iter) {
    // -------- 1) Detect the worst bound violation. ------------------
    double worst_violation = 0.0;
    int    worst_idx       = -1;
    int    worst_sign      = 0;
    for (int i = 0; i < 4; ++i) {
      if (pin[i] != 0) continue;
      if (v(i) > u_max + 1e-9) {
        const double vio = v(i) - u_max;
        if (vio > worst_violation) { worst_violation = vio; worst_idx = i; worst_sign = +1; }
      } else if (v(i) < u_min - 1e-9) {
        const double vio = u_min - v(i);
        if (vio > worst_violation) { worst_violation = vio; worst_idx = i; worst_sign = -1; }
      }
    }

    if (worst_idx < 0) {
      const Eigen::Matrix<double, 4, 1> mu = lambda;
      bool removed = false;
      for (int i = 0; i < 4; ++i) {
        if (pin[i] == 0) continue;
        const double bi_mu = Beff.col(i).dot(mu);
        const double dual  = v(i) - bi_mu;   // grad-of-lagrangian balance
        if (pin[i] == +1 && dual <  -kEpsKKT) { pin[i] = 0; removed = true; break; }
        if (pin[i] == -1 && dual >   kEpsKKT) { pin[i] = 0; removed = true; break; }
      }
      if (!removed) {
        if (converged) *converged = true;
        return v;
      }
    } else {
      pin[worst_idx] = worst_sign;
    }

    // -------- 2) Solve the reduced KKT system. ----------------------
    Eigen::Matrix<double, 4, 1> rhs = tau_des;
    for (int i = 0; i < 4; ++i) {
      if (pin[i] == +1) { v(i) = u_max; rhs -= Beff.col(i) * u_max; }
      if (pin[i] == -1) { v(i) = u_min; rhs -= Beff.col(i) * u_min; }
    }

    std::vector<int> free_idx;
    for (int i = 0; i < 4; ++i) if (pin[i] == 0) free_idx.push_back(i);

    if (free_idx.empty()) {
      if (converged) *converged = true;
      return v;
    }

    Eigen::MatrixXd Bfree(4, free_idx.size());
    for (std::size_t k = 0; k < free_idx.size(); ++k) {
      Bfree.col(k) = Beff.col(free_idx[k]);
    }

    const Eigen::Matrix<double, 4, 4> M = Bfree * Bfree.transpose() + 1e-6 * Eigen::Matrix<double, 4, 4>::Identity();
    lambda = M.ldlt().solve(rhs);

    Eigen::VectorXd vfree = Bfree.transpose() * lambda;
    for (std::size_t k = 0; k < free_idx.size(); ++k) {
      v(free_idx[k]) = vfree(k);
    }
  }

  return v;
}

// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::allocate(const WrenchVec & tau_des,
                                     double u_min, double u_max,
                                     int * status_out) const {
  std::ostringstream rep;

  // Step 1: weighted pseudo-inverse (priority-aware, no constraints).
  ControlVec u = pseudo_inverse(tau_des);

  const bool saturates =
      (u.array() > u_max + 1e-6).any() || (u.array() < u_min - 1e-6).any();

  if (!saturates) {
    if (status_out) *status_out = 0;
    rep << "pinv OK   |u|_inf=" << u.lpNorm<Eigen::Infinity>();
    report_ = rep.str();
    return u;
  }

  // Step 2: QP re-allocation with box constraints.
  bool ok = false;
  ControlVec u_qp = solve_qp(tau_des, u_min, u_max, &ok);

  // Safety clamp
  for (int i = 0; i < 4; ++i) {
    u_qp(i) = std::clamp(u_qp(i), u_min, u_max);
  }

  if (status_out) *status_out = ok ? 1 : 2;
  rep << (ok ? "QP OK   " : "QP best-effort ")
      << " |u_pinv|_inf=" << u.lpNorm<Eigen::Infinity>()
      << " |u_qp|_inf="   << u_qp.lpNorm<Eigen::Infinity>();
  report_ = rep.str();
  return u_qp;
}

// ---------------------------------------------------------------------------
WrenchVec ThrustAllocator::actual_wrench(const ControlVec & u_cmd) const {
  ControlVec u_eff;
  for (int i = 0; i < 4; ++i) u_eff(i) = fault_[i] * u_cmd(i);
  return B_ * u_eff;
}

}  // namespace auv_control
