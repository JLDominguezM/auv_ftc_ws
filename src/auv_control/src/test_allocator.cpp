// ============================================================================
//  auv_control/test_allocator.cpp
//
//  Off-line sanity check for the allocator + T-S fuzzy controller.
//  No ROS graph needed: just runs three scenarios from the paper and prints
//  key quantities so you can compare against Figures 5-11.
//
//  Scenario A : healthy          — should match "fuzzy input signal" (Fig 5)
//  Scenario B : abrupt fault u1  — should match Figs 6-7 (pinv recovery)
//  Scenario C : same fault + very tight limits — exercises the QP (Figs 10-11)
// ============================================================================
#include <array>
#include <iomanip>
#include <iostream>

#include "auv_control/auv_params.hpp"
#include "auv_control/ts_fuzzy.hpp"
#include "auv_control/thrust_allocator.hpp"

using namespace auv_control;

static void print_vec(const char * label, const Eigen::VectorXd & v) {
  std::cout << std::setw(16) << std::left << label;
  std::cout << std::fixed << std::setprecision(3);
  for (int i = 0; i < v.size(); ++i) std::cout << std::setw(8) << v(i);
  std::cout << "\n";
}

static void run(const std::string & title,
                const std::array<double, kNumThrusters> & faults,
                double u_min, double u_max) {
  std::cout << "\n==== " << title << " ====\n";

  AllocParams      geom;
  ThrustAllocator  alloc(geom);
  TSFuzzyController fuzzy;

  alloc.set_fault_factors(faults);

  // Operating point: cruising at 0.8 m/s surge, +0.05 rad/s yaw rate.
  StateVec x;      x     << 0.8, 0.0, 0.0, 0.0, 0.05;
  StateVec x_ref;  x_ref << 0.8, 0.0, 0.0, 0.0, 0.00;   // command "stop turning"

  const WrenchVec  tau_des  = fuzzy.compute(x, x_ref);

  int status = 0;
  const ControlVec u_cmd    = alloc.allocate(tau_des, u_min, u_max, &status);
  const WrenchVec  tau_act  = alloc.actual_wrench(u_cmd);

  print_vec("fault factors",
            Eigen::Map<const Eigen::Matrix<double, kNumThrusters, 1>>(faults.data()));
  print_vec("tau_des",       tau_des);
  print_vec("u_cmd (alloc)", u_cmd);
  print_vec("tau_actual",    tau_act);
  std::cout << "status = " << status
            << "   (0=pinv, 1=QP OK, 2=QP best-effort)\n"
            << "report : " << alloc.last_report() << "\n";
  std::cout << "tau error norm = "
            << (tau_des - tau_act).norm() << "\n";
}

int main() {
  //                       T1   T2   T3   T4   T5   T6
  run("A  Healthy",       {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, -50.0, 50.0);
  run("B  Kill T1 (horizontal BR)",
                          {0.0, 1.0, 1.0, 1.0, 1.0, 1.0}, -50.0, 50.0);
  run("C  Kill T1+T3 (both starboard horizontals)",
                          {0.0, 1.0, 0.0, 1.0, 1.0, 1.0}, -50.0, 50.0);
  run("D  Kill T5 (bow vertical)",
                          {1.0, 1.0, 1.0, 1.0, 0.0, 1.0}, -50.0, 50.0);
  run("E  Tight bounds + T1 dead (forces QP)",
                          {0.0, 1.0, 1.0, 1.0, 1.0, 1.0}, -10.0, 10.0);
  return 0;
}
