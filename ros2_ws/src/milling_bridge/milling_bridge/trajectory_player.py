#!/usr/bin/env python3
"""Play a Milling_Control trajectory folder on a FollowJointTrajectory action server.

Safety defaults (docs/instructions.md §三-6): ``dry_run`` is True (nothing is sent) and
``speed_scale`` is 0.1 (ten times slower than programmed). ``speed_scale`` above 1.0 needs
``allow_fast:=true`` (only for simulation / mock hardware).

    ros2 run milling_bridge trajectory_player --ros-args \
        -p trajectory_dir:=/path/to/trajectory -p dry_run:=false -p speed_scale:=0.2
"""
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .traj_file import read_trajectory


def _dur(sec: float) -> Duration:
    s = int(math.floor(sec))
    return Duration(sec=s, nanosec=int(round((sec - s) * 1e9)))


class TrajectoryPlayer(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_player")
        self.declare_parameter("trajectory_dir", "")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("speed_scale", 0.1)
        self.declare_parameter("allow_fast", False)
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("move_to_start", True)
        self.declare_parameter("start_max_joint_speed", 0.2)  # rad/s for the approach move
        self.declare_parameter("start_min_duration", 3.0)
        self.declare_parameter("lead_in", 0.5)  # s before the first point
        self.declare_parameter("goal_time_tolerance", 0.5)
        self.declare_parameter("server_timeout", 15.0)
        self.events = self.create_publisher(String, "/milling_bridge/events", 10)
        self._js: JointState | None = None
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10)

    def _on_js(self, msg: JointState) -> None:
        self._js = msg

    def _event(self, kind: str, **kw) -> None:
        now = self.get_clock().now()
        payload = {"event": kind, "stamp": now.nanoseconds * 1e-9, **kw}
        self.events.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"event {kind}: {kw}")

    def run(self) -> int:
        p = {k: self.get_parameter(k).value for k in ("trajectory_dir", "dry_run", "speed_scale", "allow_fast", "action_name", "move_to_start", "start_max_joint_speed", "start_min_duration", "lead_in", "goal_time_tolerance", "server_timeout")}
        if not p["trajectory_dir"]:
            self.get_logger().error("trajectory_dir parameter is required")
            return 2
        traj = read_trajectory(p["trajectory_dir"])
        scale = float(p["speed_scale"])
        if scale <= 0:
            self.get_logger().error("speed_scale must be > 0")
            return 2
        if scale > 1.0 and not p["allow_fast"]:
            self.get_logger().warning(f"speed_scale {scale} > 1 clamped to 1.0 (set allow_fast:=true for simulation only)")
            scale = 1.0
        q_first = traj.q[0]
        self.get_logger().info(
            f"trajectory {p['trajectory_dir']}: {traj.n} points, {traj.duration:.1f} s programmed, "
            f"{traj.duration / scale:.1f} s at speed_scale {scale}; joints {traj.joint_names}; "
            f"gcode {traj.meta.get('gcode')} md5 {str(traj.meta.get('gcode_md5'))[:8]}"
        )
        if p["dry_run"]:
            self.get_logger().warning("DRY RUN: nothing sent. Re-run with -p dry_run:=false to execute.")
            self.get_logger().info(f"first point (deg): {[round(math.degrees(v), 2) for v in q_first]}")
            return 0

        # --- current joint state
        t0 = time.time()
        while self._js is None or not set(traj.joint_names) <= set(self._js.name):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > p["server_timeout"]:
                self.get_logger().error("no joint_states with the trajectory's joints received")
                return 3
        idx = [self._js.name.index(n) for n in traj.joint_names]
        q_now = [self._js.position[i] for i in idx]

        client = ActionClient(self, FollowJointTrajectory, p["action_name"])
        if not client.wait_for_server(timeout_sec=p["server_timeout"]):
            self.get_logger().error(f"action server {p['action_name']} not available")
            return 3

        # --- approach: current -> first point
        if p["move_to_start"]:
            dq = max(abs(a - b) for a, b in zip(q_now, q_first))
            dur = max(float(p["start_min_duration"]), dq / float(p["start_max_joint_speed"]))
            self.get_logger().info(f"move_to_start: max joint delta {math.degrees(dq):.2f} deg over {dur:.1f} s")
            jt = JointTrajectory(joint_names=traj.joint_names)
            jt.points = [JointTrajectoryPoint(positions=q_now, time_from_start=_dur(0.0)), JointTrajectoryPoint(positions=q_first, time_from_start=_dur(dur))]
            self._event("approach_sent", duration=dur)
            code = self._send(client, jt, p["goal_time_tolerance"])
            if code != 0:
                return 4

        # --- main trajectory, time-scaled
        jt = JointTrajectory(joint_names=traj.joint_names)
        lead = float(p["lead_in"])
        jt.points = [JointTrajectoryPoint(positions=q, time_from_start=_dur(lead + (t - traj.t[0]) / scale)) for t, q in zip(traj.t, traj.q)]
        self._event("main_sent", n_points=traj.n, duration=lead + traj.duration / scale, speed_scale=scale, trajectory_dir=p["trajectory_dir"])
        code = self._send(client, jt, p["goal_time_tolerance"])
        self._event("main_done", error_code=code)
        return 0 if code == 0 else 5

    def _send(self, client: ActionClient, jt: JointTrajectory, goal_tol: float) -> int:
        goal = FollowJointTrajectory.Goal(trajectory=jt, goal_time_tolerance=_dur(goal_tol))
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("goal rejected")
            return -1
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        self.get_logger().info(f"result error_code {result.error_code} {result.error_string!r}")
        return int(result.error_code)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
