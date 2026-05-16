#!/usr/bin/env bash
# ============================================================================
#  run_demo.sh — Single-session FTC demo.
#
#  Launches the simulation ONCE and runs three scenarios back-to-back by
#  starting/stopping rosbags and injecting faults via the
#  /auv/inject_fault service. Avoids the Gazebo-Classic restart bug that
#  hangs world loading on second start.
#
#  Output:
#      results/A_healthy/    bag + figs (no faults)
#      results/B_T1_dead/    bag + figs (T1 dies at t=20s of that bag)
#      results/C_T1_T3_dead/ bag + figs (T1 and T3 die at t=20s of that bag)
# ============================================================================
set -e

WS="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$WS/results"
SEG_SECS=60        # length of each scenario's recording
T_FAULT=20         # seconds into the bag at which to inject the fault
SETTLE=10          # seconds to wait between scenarios so dynamics damp out

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ---------- 1. Clean up any leftover sim and start ONE Gazebo session ----
echo ">> killing any prior sim"
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f auv_controller_node 2>/dev/null || true
pkill -9 -f "ros2 bag" 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 2

mkdir -p "$OUT"

echo ">> launching simulation"
ros2 launch auv_bringup full_simulation.launch.py >"$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

# Wait for /auv/odom to start publishing (= AUV spawned, controller running).
echo ">> waiting for /auv/odom..."
for i in $(seq 1 60); do
  if ros2 topic info /auv/odom 2>/dev/null | grep -q "Publisher count: 1"; then
    if timeout 2 ros2 topic echo --once /auv/odom >/dev/null 2>&1; then
      echo "   AUV alive after ${i}s"
      break
    fi
  fi
  sleep 1
done

# Give it a few more seconds to start moving on the trajectory.
sleep 5

# ---------- 2. Helper to call inject_fault ------------------------------
inject () {
  local tid="$1" factor="$2"
  ros2 service call /auv/inject_fault auv_control/srv/InjectFault \
       "{thruster_id: $tid, fault_factor: $factor, fault_type: abrupt, ramp_seconds: 0.0}" \
       >/dev/null
}

heal_all () {
  for t in 1 2 3 4 5 6; do inject $t 1.0; done
}

# ---------- 3. Run a scenario (bag + faults + plots) --------------------
run_scenario () {
  local name="$1"; shift
  local desc="$1"; shift
  local dir="$OUT/$name"

  echo "=========================================================="
  echo " $name : $desc"
  echo "=========================================================="
  rm -rf "$dir"
  mkdir -p "$dir/figs"

  heal_all
  sleep 1

  # Start recording (in background).
  ros2 bag record -o "$dir/bag" \
       /auv/virtual_u /auv/tau_des /auv/tau_actual /auv/fault_status \
       /auv/odom /auv/target_pose \
       >"$dir/bag.log" 2>&1 &
  local BAG_PID=$!

  # Phase 1: healthy run for T_FAULT seconds.
  sleep "$T_FAULT"

  # Phase 2: inject the requested faults.
  for tid in "$@"; do
    echo "  -> kill T$tid"
    inject "$tid" 0.0
  done

  # Phase 3: let the rest of the scenario play out.
  sleep $((SEG_SECS - T_FAULT))

  # Stop the bag (SIGINT so it flushes properly).
  kill -INT "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true

  # Heal the faults so the next scenario starts clean.
  heal_all

  # Plot.
  if [ -n "$*" ]; then
    python3 "$WS/src/auv_control/scripts/plot_from_bag.py" \
      --bag "$dir/bag" --out "$dir/figs" --t-fault "$T_FAULT" \
      >"$dir/plot.log" 2>&1 || true
  else
    python3 "$WS/src/auv_control/scripts/plot_from_bag.py" \
      --bag "$dir/bag" --out "$dir/figs" \
      >"$dir/plot.log" 2>&1 || true
  fi

  echo "  done -> $dir/figs/"
  echo ">> settling ${SETTLE}s before next scenario"
  sleep "$SETTLE"
}

# ---------- 4. Run the three scenarios in this session ------------------
run_scenario "A_healthy"    "baseline, no faults"
run_scenario "B_T1_dead"    "kill T1 at t=${T_FAULT}s"          1
run_scenario "C_T1_T3_dead" "kill T1+T3 at t=${T_FAULT}s"       1 3

# ---------- 5. Tear down the simulation ---------------------------------
echo ">> tearing down simulation"
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 3
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f auv_controller_node 2>/dev/null || true

echo "=========================================================="
echo " All scenarios complete. Results in: $OUT"
echo "=========================================================="
