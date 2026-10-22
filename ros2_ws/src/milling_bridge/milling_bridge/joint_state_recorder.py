#!/usr/bin/env python3
"""Record /joint_states (and the player's events) to CSV + JSON for sim-vs-real comparison.

    ros2 run milling_bridge joint_state_recorder --ros-args -p output_dir:=/path/to/session
Stops with Ctrl-C (files are flushed continuously).
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class JointStateRecorder(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_recorder")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("joints", ["J1", "J2", "J3", "J4", "J5", "J6"])
        self.declare_parameter("joint_states_topic", "/joint_states")
        out = self.get_parameter("output_dir").value
        if not out:
            raise SystemExit("output_dir parameter is required")
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.joints = list(self.get_parameter("joints").value)
        self.f = (self.out / "joint_states.csv").open("w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", *self.joints, *[f"{j}_vel" for j in self.joints]])
        self.events: list[dict] = []
        self.n = 0
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self._on_js, 50)
        self.create_subscription(String, "/milling_bridge/events", self._on_event, 10)
        self.get_logger().info(f"recording {self.joints} to {self.out}")

    def _on_js(self, msg: JointState) -> None:
        if not set(self.joints) <= set(msg.name):
            return
        idx = [msg.name.index(j) for j in self.joints]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = [msg.position[i] for i in idx]
        vel = [msg.velocity[i] if len(msg.velocity) == len(msg.name) else float("nan") for i in idx]
        self.w.writerow([f"{t:.6f}", *[f"{v:.9g}" for v in pos], *[f"{v:.9g}" for v in vel]])
        self.n += 1
        if self.n % 100 == 0:
            self.f.flush()

    def _on_event(self, msg: String) -> None:
        try:
            self.events.append(json.loads(msg.data))
        except json.JSONDecodeError:
            self.events.append({"event": "raw", "data": msg.data})
        (self.out / "events.json").write_text(json.dumps(self.events, indent=2))

    def close(self) -> None:
        self.f.flush()
        self.f.close()
        (self.out / "events.json").write_text(json.dumps(self.events, indent=2))
        self.get_logger().info(f"wrote {self.n} joint states, {len(self.events)} events")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
