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
using ControlVec = Eigen::Matrix<double, 6, 1>;   // [T1..T6] 6-thruster X-6 layout
using WrenchVec  = Eigen::Matrix<double, 4, 1>;   // [tau_x tau_z tau_m tau_n] (No Sway)

// Number of physical actuators (must match ControlVec size).
constexpr int kNumThrusters = 6;

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
// X-6 layout:
//   T1..T4 horizontal thrusters in the 4 corners of the hull, all pointing
//   along +x_body. Lateral offset b (port/stbd), longitudinal offset a
//   (bow/stern). Sign of yaw moment alternates with port/stbd.
//   T5, T6 vertical thrusters at +/- longitudinal offset f from CG, both
//   pointing +z_body. Pitch moment alternates bow/stern.
struct AllocParams {
  double b = 0.30;   // lateral arm of horizontals (yaw moment arm)
  double f = 0.40;   // longitudinal arm of verticals (pitch moment arm)
};

// Build the 4x6 configuration matrix B.
//   rows : [tau_x, tau_z, tau_m, tau_n] (Fx, Fz, My, Mz)
//   cols : [T1(BR), T2(BL), T3(SR), T4(SL), T5(VB), T6(VS)]
inline Eigen::Matrix<double, 4, 6> build_B(const AllocParams & g) {
  Eigen::Matrix<double, 4, 6> B;
  B.setZero();

  // tau_x: all 4 horizontal thrusters push along +x_body.
  B(0, 0) = 1.0;  B(0, 1) = 1.0;  B(0, 2) = 1.0;  B(0, 3) = 1.0;

  // tau_z: only the 2 vertical thrusters.
  B(1, 4) = 1.0;  B(1, 5) = 1.0;

  // tau_m (pitch): vertical bow vs stern create opposite-sign pitch moment.
  B(2, 4) = -g.f;
  B(2, 5) = +g.f;

  // tau_n (yaw): horizontal port/stbd create opposite-sign yaw moment.
  //   T1 BR -> -b   T2 BL -> +b   T3 SR -> -b   T4 SL -> +b
  B(3, 0) = -g.b;  B(3, 1) = +g.b;  B(3, 2) = -g.b;  B(3, 3) = +g.b;

  return B;
}

}  // namespace auv_control

#endif  // AUV_CONTROL__AUV_PARAMS_HPP_
