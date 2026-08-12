#!/usr/bin/env python3
"""Continuously publish RobotCmd.mode=3 (G_COMPENSATION) so the arm is drag-able."""
from __future__ import annotations

import time

import rclpy
from arx5_arm_msg.msg import RobotCmd, RobotStatus
from rclpy.node import Node


class GravityBG(Node):
    def __init__(self) -> None:
        super().__init__("x5a_force_gravity_bg")
        self.q = None
        self.g = 0.0
        self.pub = self.create_publisher(RobotCmd, "arm_cmd", 10)
        self.create_subscription(RobotStatus, "arm_status", self.on_status, 10)
        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().warn("waiting for /arm_status ...")

    def on_status(self, msg: RobotStatus) -> None:
        self.q = [float(msg.joint_pos[i]) for i in range(6)]
        if len(msg.joint_pos) > 6:
            self.g = float(msg.joint_pos[6])

    def on_timer(self) -> None:
        if self.q is None:
            return
        m = RobotCmd()
        m.header.stamp = self.get_clock().now().to_msg()
        m.end_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(6):
            m.joint_pos[i] = float(self.q[i])
        m.gripper = float(self.g)
        m.mode = 3  # G_COMPENSATION
        self.pub.publish(m)


def main() -> None:
    rclpy.init()
    node = GravityBG()
    t0 = time.time()
    while node.q is None and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.q is None:
        node.get_logger().error("no /arm_status — is X5Controller running?")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)
    node.get_logger().warn(
        f"GRAVITY ON (mode=3). q={[round(v, 3) for v in node.q]}. Drag the arm. Ctrl+C to stop."
    )
    print("*** GRAVITY ON (mode=3) — drag the arm ***", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # timeout / external shutdown should not crash noisily
        print(f"spin ended: {type(exc).__name__}: {exc}", flush=True)
    finally:
        # hold on exit if context still valid
        try:
            if node.q is not None and rclpy.ok():
                for _ in range(20):
                    m = RobotCmd()
                    m.header.stamp = node.get_clock().now().to_msg()
                    m.end_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    for i in range(6):
                        m.joint_pos[i] = float(node.q[i])
                    m.gripper = float(node.g)
                    m.mode = 5
                    node.pub.publish(m)
                    time.sleep(0.02)
                print("*** HOLD (mode=5) on exit ***", flush=True)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
