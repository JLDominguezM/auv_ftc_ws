#!/usr/bin/env python3
# ============================================================================
#  inject_fault_cli.py
#
#  Convenience wrapper around the /auv/inject_fault service.
#
#  Examples:
#      # At t = 150 s in your simulation, cause total loss of u1:
#      ros2 run auv_control inject_fault_cli.py --id 1 --factor 0.0
#
#      # Slow degradation of rudder u4 over 30 s down to 20% effectiveness:
#      ros2 run auv_control inject_fault_cli.py --id 4 --factor 0.2 \
#           --type slow --ramp 30.0
# ============================================================================
import argparse
import sys

import rclpy
from rclpy.node import Node

from auv_control.srv import InjectFault


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--id",    type=int,   required=True, choices=[1, 2, 3, 4, 5, 6],
                   help="thruster id (1..6) in the X-6 layout (T1..T4 horiz, T5/T6 vert)")
    p.add_argument("--factor", type=float, required=True,
                   help="target fault factor f_i in [0, 1] (0 = total loss)")
    p.add_argument("--type",   choices=["abrupt", "slow"], default="abrupt")
    p.add_argument("--ramp",   type=float, default=0.0,
                   help="ramp duration in seconds (used only for --type slow)")
    args = p.parse_args()

    rclpy.init()
    node = Node("inject_fault_cli")
    cli  = node.create_client(InjectFault, "/auv/inject_fault")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/auv/inject_fault service not available")
        rclpy.shutdown()
        sys.exit(1)

    req = InjectFault.Request()
    req.thruster_id  = args.id
    req.fault_factor = args.factor
    req.fault_type   = args.type
    req.ramp_seconds = args.ramp

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    rsp = fut.result()
    if rsp and rsp.accepted:
        node.get_logger().info(f"accepted: {rsp.message}")
    else:
        node.get_logger().error(
            f"rejected: {rsp.message if rsp else '<no response>'}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
