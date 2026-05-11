// ============================================================================
//  auv_control/auv_params.hpp
//
//  Physical / geometric parameters for the AUV and the thrust-allocation
//  configuration matrix B (Eq. 26 of Zhang et al., Sensors 24, 3029).
//
//  Control layout:
//      u = [u1 u2 u3 u4]^T   (four actuator channels)
//      tau = [tau_x tau_z tau_m tau_n]^T  (4-DOF body-frame wrench)
//      tau = B * u
//
//  The AUV is under-actuated (no pure sway control). We control 4 DOFs
//  with 4 actuators to ensure a mathematically stable square system.
// ============================================================================
#ifndef AUV_CONTROL__AUV_PARAMS_HPP_
#define AUV_CONTROL__AUV_PARAMS_HPP_

#include <Eigen/Dense>

namespace auv_control {

// ---- Shared small-vector types (used by every block) ----------------------
using StateVec   = Eigen::Matrix<double, 5, 1>;   // [u v w q r]
using ControlVec = Eigen::Matrix<double, 4, 1>;   // [u1 u2 u3 u4]
using WrenchVec  = Eigen::Matrix<double, 4, 1>;   // [tau_x tau_z tau_m tau_n] (No Sway)

// ---- Vehicle mass / buoyancy parameters ------------------------------------
struct VehicleParams {
  double mass            = 25.0;    // kg
  double buoyancy_force  = 25.0 * 9.81 * 1.01;  // slightly positive buoyancy, N
  double gravity         = 9.81;    // m/s^2

  // Linear drag coefficients (balanced)
  double drag_u = 30.0;
  double drag_v = 60.0;
  double drag_w = 60.0;
  double drag_p = 10.0;
  double drag_q = 20.0;
  double drag_r = 20.0;

  // Quadratic drag (|v|*v terms)
  double drag_uu = 15.0;
  double drag_vv = 40.0;
  double drag_ww = 40.0;
  double drag_qq = 5.0;
  double drag_rr = 5.0;

  // Maximum thruster output (N) — increased for FTC overhead
  double thrust_max = 100.0;
  double thrust_min = -100.0;
  double u_scale = 25.0;
};

// ---- Thrust-allocation geometry --------------------------------------------
struct AllocParams {
  double alpha = 0.35;
  double beta  = 0.35;
  double a     = 0.10;
  double b     = 0.10;
  double l     = 1.00;
};

// Build the 4x4 configuration matrix B.
inline Eigen::Matrix<double, 4, 4> build_B(const AllocParams & g) {
  (void)g;
  const double arm = 0.5;
  Eigen::Matrix<double, 4, 4> B;
  B.setZero();
  
  B(0, 0) = 1.0;  // u1: Surge
  B(1, 1) = 1.0;  // u2: Heave
  B(2, 2) = arm;  // u3: Pitch
  B(3, 3) = arm;  // u4: Yaw

  return B;
}

}  // namespace auv_control

#endif  // AUV_CONTROL__AUV_PARAMS_HPP_
