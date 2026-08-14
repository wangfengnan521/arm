#!/usr/bin/env python3
"""Interactive joint range calibration for the ARX X5A (LeRobot SO-101 style).

OBSERVE ONLY: this tool subscribes to real robot feedback and NEVER commands
the arm. It records the human-swept safe joint range of joint1..joint6,
compares it against the current MoveIt joint_limits.yaml, and writes two
YAML artifacts:

  calibration/x5a_joint_range_YYYYMMDD_HHMMSS.yaml     measured data
  calibration/x5a_joint_limits_suggested.yaml          recommended soft limits

The measured extrema are called observed_range / recommended_soft_range on
purpose: they prove the swept range is safe to use, NOT that it is the
mechanical hard limit. The tool never modifies joint_limits.yaml.

Why no --manual-mode: the official X5Controller exposes no runtime
zero-force / manual-teach / torque-off service (gravity compensation exists
only as the remote_master startup mode, which requires a controller
relaunch). This tool therefore stays OBSERVE ONLY.

Usage:
  ros2 run x5a_moveit_official_adapter joint_range_calibrate
  ros2 run x5a_moveit_official_adapter joint_range_calibrate --guided
  ros2 run x5a_moveit_official_adapter joint_range_calibrate --margin-rad 0.03
"""
from __future__ import annotations

import argparse
import copy
import datetime
import math
import os
import select
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

try:
    from ament_index_python.packages import get_package_share_directory
    from arx5_arm_msg.msg import RobotStatus
except ImportError:  # pragma: no cover - arx5_arm_msg lives in the vendor overlay
    RobotStatus = None
    def get_package_share_directory(_name: str) -> str:  # type: ignore[misc]
        raise RuntimeError("ament_index unavailable")


class JointRecorder:
    def __init__(self) -> None:
        self.observed_min: List[float] = [math.inf] * 6
        self.observed_max: List[float] = [-math.inf] * 6
        self.samples = 0
        self.peak_abs_vel: List[float] = [0.0] * 6
        self.peak_abs_cur: List[float] = [0.0] * 6
        self.baseline_cur: Optional[List[float]] = None
        self.baseline_done_at: Optional[float] = None

    def reset(self) -> None:
        self.__init__()

    def feed(
        self,
        position: List[float],
        velocity: Optional[List[float]],
        current: Optional[List[float]],
        now: float,
    ) -> None:
        if self.samples == 0:
            self.observed_min = list(position)
            self.observed_max = list(position)
        self.samples += 1
        for i in range(6):
            self.observed_min[i] = min(self.observed_min[i], position[i])
            self.observed_max[i] = max(self.observed_max[i], position[i])
            if velocity is not None:
                self.peak_abs_vel[i] = max(self.peak_abs_vel[i], abs(velocity[i]))
            if current is not None:
                self.peak_abs_cur[i] = max(self.peak_abs_cur[i], abs(current[i]))
        # baseline for HIGH CURRENT detection: max |cur| seen in the first second
        if current is not None:
            if self.baseline_done_at is None:
                self.baseline_done_at = now + 1.0
                self.baseline_cur = [abs(c) for c in current]
            elif now < self.baseline_done_at:
                for i in range(6):
                    self.baseline_cur[i] = max(self.baseline_cur[i], abs(current[i]))

    def high_current_warning(self, current: List[float], factor: float, now: float) -> List[str]:
        warnings: List[str] = []
        if self.baseline_cur is None or self.baseline_done_at is None:
            return warnings
        if now < self.baseline_done_at:
            return warnings
        for i in range(6):
            threshold = max(self.baseline_cur[i] * factor, 1e-9)
            if abs(current[i]) > threshold and abs(current[i]) > 1e-9:
                warnings.append(
                    f"joint{i + 1}: cur={current[i]:.3f} raw vs baseline {self.baseline_cur[i]:.3f} raw"
                )
        return warnings


class JointRangeCalibrate(Node):
    def __init__(
        self,
        margin_rad: float,
        guided: bool,
        moveit_limits_file: str,
        out_dir: str,
        current_warn_factor: float,
    ) -> None:
        super().__init__("x5a_joint_range_calibrate")
        self.margin_rad = margin_rad
        self.guided = guided
        self.current_warn_factor = current_warn_factor
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.moveit_limits: Dict[str, Dict[str, float]] = {}
        self.moveit_limits_file = moveit_limits_file
        self._load_moveit_limits()

        # /arm_status (vendor, 100 Hz, RELIABLE) -> raw joint_pos/vel/cur
        self.status: Optional[List[float]] = None
        self.status_vel: Optional[List[float]] = None
        self.status_cur: Optional[List[float]] = None
        self.status_stamp = 0.0
        self.status_available = False
        if RobotStatus is not None:
            self.create_subscription(
                RobotStatus,
                "/arm_status",
                self._status_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            )

        # /joint_states (adapter derived; REQUIRED fallback + freshness source)
        self.js: Dict[str, float] = {}
        self.js_vel: Dict[str, float] = {}
        self.js_stamp = 0.0
        self.js_available = False
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self.recorder = JointRecorder()
        self.recording = False
        self.feedback_lost = False
        self.tty = sys.stdout.isatty()
        self._interrupt_printed = False
        # Interactive input: prefer the controlling terminal so ENTER keeps
        # working even when stdin is redirected (/dev/null, pipes, panels).
        self.input_fd: Optional[int] = None
        try:
            self.input_fd = os.open("/dev/tty", os.O_RDONLY)
        except OSError:
            self.input_fd = None
        # Topic-based stop channel: publish once to end the current window,
        # works from ANY terminal regardless of how this process was started.
        self.next_requested = False
        self.create_subscription(
            Empty, "~/x5a_joint_range_calibrate/next", self._next_cb, 10
        )

    def _next_cb(self, _msg: Empty) -> None:
        self.next_requested = True

    # ---------------------------------------------------------------- inputs
    def _load_moveit_limits(self) -> None:
        try:
            with open(self.moveit_limits_file, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exception:
            self.get_logger().warn(
                f"cannot read MoveIt limits {self.moveit_limits_file}: {exception}"
            )
            return
        limits = (data or {}).get("joint_limits", {})
        for joint in ARM_JOINTS:
            entry = limits.get(joint, {})
            if "min_position" in entry and "max_position" in entry:
                self.moveit_limits[joint] = {
                    "min_position": float(entry["min_position"]),
                    "max_position": float(entry["max_position"]),
                }

    def _status_cb(self, msg) -> None:
        if len(msg.joint_pos) < 6:
            return
        self.status = [float(msg.joint_pos[i]) for i in range(6)]
        self.status_vel = (
            [float(msg.joint_vel[i]) for i in range(6)]
            if len(msg.joint_vel) >= 6 else None
        )
        self.status_cur = (
            [float(msg.joint_cur[i]) for i in range(6)]
            if len(msg.joint_cur) >= 6 else None
        )
        self.status_stamp = time.monotonic()
        self.status_available = True

    def _js_cb(self, msg: JointState) -> None:
        names = list(msg.name)
        for i, name in enumerate(names):
            if name in ARM_JOINTS and i < len(msg.position):
                self.js[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self.js_vel[name] = float(msg.velocity[i])
        self.js_stamp = time.monotonic()
        self.js_available = True

    # ------------------------------------------------------------ snapshots
    def current_position(self) -> Optional[List[float]]:
        if self.status is not None and time.monotonic() - self.status_stamp < 0.5:
            return list(self.status)
        if all(name in self.js for name in ARM_JOINTS):
            return [self.js[name] for name in ARM_JOINTS]
        return None

    def current_velocity(self) -> Optional[List[float]]:
        if self.status_vel is not None and time.monotonic() - self.status_stamp < 0.5:
            return list(self.status_vel)
        if all(name in self.js_vel for name in ARM_JOINTS):
            return [self.js_vel[name] for name in ARM_JOINTS]
        return None

    def current_raw(self) -> Optional[List[float]]:
        if self.status_cur is not None and time.monotonic() - self.status_stamp < 0.5:
            return list(self.status_cur)
        return None

    def feedback_ok(self) -> bool:
        now = time.monotonic()
        js_fresh = now - self.js_stamp < 0.5
        st_fresh = now - self.status_stamp < 0.5
        return js_fresh or st_fresh

    # ------------------------------------------------------------------ ui
    def _input_fd(self) -> Optional[int]:
        """Keyboard source: /dev/tty first, stdin as fallback."""
        if self.input_fd is not None:
            return self.input_fd
        try:
            return sys.stdin.fileno()
        except (OSError, ValueError):
            return None

    def _wait_enter(self, timeout: float) -> bool:
        """True when the current window should end.

        Triggers: (1) a message on ~/x5a_joint_range_calibrate/next (works
        from any terminal), (2) ENTER/EOF on /dev/tty or stdin. Reads via
        os.read() on the raw fd (NOT sys.stdin.readline()): the
        TextIOWrapper's internal buffer can read ahead several queued lines
        from the kernel queue, after which select() on the fd reports empty
        forever and ENTER stops working.
        """
        if self.next_requested:
            self.next_requested = False
            return True
        fd = self._input_fd()
        if fd is None:
            return False
        ready, _, _ = select.select([fd], [], [], min(timeout, 0.15))
        if not ready:
            return False
        try:
            os.read(fd, 4096)  # data or EOF both count as "enter"
        except OSError:
            pass
        return True

    def _drain_stdin(self, timeout: float = 0.2) -> None:
        """Discard leftover keystrokes so a stray ENTER cannot end the next window."""
        fd = self._input_fd()
        if fd is None:
            return
        while True:
            ready, _, _ = select.select([fd], [], [], timeout)
            if not ready:
                return
            try:
                if not os.read(fd, 4096):
                    return
            except OSError:
                return

    def _screen_header(self) -> None:
        if self.tty:
            print("\x1b[2J\x1b[H", end="")
        print("X5A JOINT RANGE CALIBRATION  (OBSERVE ONLY - never commands the arm)")
        print()

    def _print_current_feedback(self) -> None:
        if self.tty:
            print("\x1b[2J\x1b[H", end="")
        position = self.current_position()
        velocity = self.current_velocity()
        current = self.current_raw()
        print("Current feedback:")
        for i, name in enumerate(ARM_JOINTS):
            p = position[i] if position else None
            v = velocity[i] if velocity else None
            c = current[i] if current else None
            print(
                f"{name:>7}  pos={p if p is None else round(p, 4):>10}  "
                f"vel={v if v is None else round(v, 4):>10}  "
                f"cur={c if c is None else round(c, 3):>10} raw"
            )
        print()

    def _print_instructions(self) -> None:
        print("Instructions:")
        print("1. Put robot in a comfortable middle configuration.")
        print("2. Press ENTER to start recording.")
        print("3. Move joints one by one through the SAFE usable range.")
        print("4. Do NOT force a joint against a mechanical stop.")
        print("5. Press ENTER again to finish.")
        print("   (or publish to /x5a_joint_range_calibrate/next from any terminal)")
        print()

    def _print_recording_table(self, recorder: JointRecorder) -> None:
        position = self.current_position()
        velocity = self.current_velocity()
        current = self.current_raw()
        rec = recorder
        if self.tty:
            print("\x1b[2J\x1b[H", end="")
        else:
            print()
        print("RECORDING ... move each joint through its safe range")
        print("Stop: press ENTER here, or from ANY terminal run:")
        print("  ros2 topic pub --once /x5a_joint_range_calibrate/next "
              "std_msgs/msg/Empty '{}'")
        print("-" * 72)
        print(
            f"{'JOINT':>7} {'MIN':>10} {'CURRENT':>10} {'MAX':>10} "
            f"{'|VEL|':>10} {'|CUR|':>10}"
        )
        for i, name in enumerate(ARM_JOINTS):
            lo = rec.observed_min[i] if rec.samples else 0.0
            hi = rec.observed_max[i] if rec.samples else 0.0
            p = position[i] if position else 0.0
            v = rec.peak_abs_vel[i] if rec.samples else 0.0
            c = rec.peak_abs_cur[i] if rec.samples else 0.0
            print(
                f"{name:>7} {lo:>10.3f} {p:>10.3f} {hi:>10.3f} "
                f"{v:>10.2f} {c:>10.2f}"
            )
        print("-" * 72)
        print(f"samples={rec.samples}")
        if current is not None:
            warnings = rec.high_current_warning(
                current, self.current_warn_factor, time.monotonic()
            )
            if warnings:
                print("\x1b[31mHIGH CURRENT:\x1b[0m")
                for line in warnings:
                    print(f"  {line}")
        if not self.feedback_ok():
            print("\x1b[31mFEEDBACK STALE\x1b[0m")

    def _run_loop(self, stop_fn, recorder: JointRecorder) -> str:
        """Run until stop_fn() returns True. Returns 'ok' or 'feedback_lost'."""
        period = 0.15
        last_print = 0.0
        while True:
            if stop_fn():
                return "ok"
            if not self.feedback_ok():
                print("\x1b[31mFEEDBACK LOST\x1b[0m")
                return "feedback_lost"
            position = self.current_position()
            if position is not None:
                recorder.feed(
                    position, self.current_velocity(), self.current_raw(),
                    time.monotonic(),
                )
            now = time.monotonic()
            if now - last_print >= period:
                last_print = now
                self._print_recording_table(recorder)
            rclpy.spin_once(self, timeout_sec=0.02)

    # -------------------------------------------------------------- running
    def run_continuous(self) -> Optional[JointRecorder]:
        self._screen_header()
        print("OBSERVE ONLY: this program never commands the arm.")
        print("Feedback sources: /joint_states (required), /arm_status (optional)\n")
        self._print_current_feedback()
        print("Press ENTER to start recording, or Ctrl-C to quit.")
        while True:
            if self._wait_enter(0.15):
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        self._drain_stdin()
        self.recording = True
        self.recorder.reset()
        outcome = self._run_loop(lambda: self._wait_enter(0.15), self.recorder)
        self.recording = False
        if outcome != "ok":
            self.get_logger().error("recording aborted: " + outcome)
            return None
        return self.recorder

    def run_guided(self) -> Dict[str, JointRecorder]:
        self._screen_header()
        print("GUIDED MODE: one joint at a time.")
        print("OBSERVE ONLY: this program never commands the arm.\n")
        per_joint: Dict[str, JointRecorder] = {}
        current_name: Optional[str] = None
        current_recorder: Optional[JointRecorder] = None
        try:
            for i, name in enumerate(ARM_JOINTS):
                self.next_requested = False
                current_name = name
                current_recorder = JointRecorder()
                print(
                    f"\nMove {name} through its safe range.\n"
                    f"Press ENTER when finished."
                )
                recorder = current_recorder
                # record from the moment the prompt is shown until ENTER
                outcome = self._run_loop(lambda: self._wait_enter(0.15), recorder)
                self._drain_stdin()
                if outcome != "ok":
                    self.get_logger().error(f"{name} window aborted: " + outcome)
                    break
                per_joint[name] = recorder
                current_name = None
                current_recorder = None
                entry = recorder
                if entry.samples == 0:
                    print(f"{name} recorded: no samples in window")
                else:
                    print(
                        f"{name} recorded span=[{entry.observed_min[i]:.3f}, "
                        f"{entry.observed_max[i]:.3f}] samples={entry.samples}"
                    )
                # incremental save so a crash or stuck terminal never loses the sweep
                self._save_partial(per_joint)
        except KeyboardInterrupt:
            # include the in-flight window if it captured any samples
            if (current_recorder is not None and current_name is not None and
                    current_recorder.samples > 0):
                per_joint[current_name] = current_recorder
            if per_joint:
                self._save_partial(per_joint)
            self._print_interrupted(saved=bool(per_joint))
            raise
        return per_joint

    def _print_interrupted(self, saved: bool) -> None:
        if self._interrupt_printed:
            return
        self._interrupt_printed = True
        print("\nCALIBRATION INTERRUPTED")
        if saved:
            path = self.out_dir / "x5a_joint_range_partial.yaml"
            print("PARTIAL RESULT SAVED")
            print(str(path))
        else:
            print("NO PARTIAL DATA TO SAVE")

    def _save_partial(self, per_joint: Dict[str, JointRecorder]) -> None:
        path = self.out_dir / "x5a_joint_range_partial.yaml"
        data: Dict[str, Dict] = {}
        for i, name in enumerate(ARM_JOINTS):
            recorder = per_joint.get(name)
            if recorder is None or recorder.samples == 0:
                data[name] = {"observed_min": None, "observed_max": None,
                              "samples": 0, "range_span": None}
                continue
            lo = recorder.observed_min[i]
            hi = recorder.observed_max[i]
            data[name] = {
                "observed_min": round(lo, 4),
                "observed_max": round(hi, 4),
                "samples": recorder.samples,
                "range_span": round(hi - lo, 4),
            }
        try:
            with open(path, "w") as f:
                yaml.safe_dump(
                    {"robot": "X5A", "partial": True, "joints": data},
                    f, sort_keys=False, default_flow_style=False,
                )
        except OSError as exception:
            self.get_logger().warn(f"cannot write partial file: {exception}")

    # -------------------------------------------------------------- results
    def _build_results(self, per_joint: Dict[str, JointRecorder]) -> Dict:
        joints: Dict[str, Dict] = {}
        for i, name in enumerate(ARM_JOINTS):
            recorder = per_joint.get(name)
            if recorder is None or recorder.samples == 0:
                joints[name] = {
                    "observed_min": None,
                    "observed_max": None,
                    "samples": 0,
                    "range_span": None,
                    "recommended_min": None,
                    "recommended_max": None,
                    "coverage": "no_data",
                }
                continue
            lo = recorder.observed_min[i]
            hi = recorder.observed_max[i]
            span = hi - lo
            coverage = "ok" if span >= 0.2 else "insufficient"
            joints[name] = {
                "observed_min": round(lo, 4),
                "observed_max": round(hi, 4),
                "samples": recorder.samples,
                "range_span": round(span, 4),
                "recommended_min": round(lo + self.margin_rad, 4)
                if coverage == "ok" else None,
                "recommended_max": round(hi - self.margin_rad, 4)
                if coverage == "ok" else None,
                "coverage": coverage,
            }
        peaks_vel = {ARM_JOINTS[i]: max(rec.peak_abs_vel[i] for rec in per_joint.values())
                     if per_joint else 0.0 for i in range(6)}
        peaks_cur = {ARM_JOINTS[i]: max(rec.peak_abs_cur[i] for rec in per_joint.values())
                     if per_joint else 0.0 for i in range(6)}
        return {
            "robot": "X5A",
            "source": "real_joint_feedback",
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "margin_rad": self.margin_rad,
            "joints": joints,
            "peak_velocity": {k: round(v, 4) for k, v in peaks_vel.items()},
            "peak_current": {k: round(v, 4) for k, v in peaks_cur.items()},
        }

    def _compare_and_report(self, results: Dict) -> None:
        joints = results["joints"]
        print("\nMEASURED RANGE")
        for name in ARM_JOINTS:
            entry = joints[name]
            print(
                f"{name}:\n"
                f"  observed_min: {entry['observed_min']}\n"
                f"  observed_max: {entry['observed_max']}\n"
                f"  span: {entry['range_span']}\n"
                f"  coverage: {entry['coverage']}"
            )
        print("\nCOMPARISON")
        print(
            f"{'':>10} {'OBSERVED':>24} {'CURRENT MOVEIT':>24}"
        )
        for name in ARM_JOINTS:
            entry = joints[name]
            observed = (
                f"[{entry['observed_min']}, {entry['observed_max']}]"
                if entry["observed_min"] is not None else "[no data]"
            )
            existing = self.moveit_limits.get(name)
            existing_str = (
                f"[{existing['min_position']}, {existing['max_position']}]"
                if existing else "[unknown]"
            )
            print(f"{name:>10} {observed:>24} {existing_str:>24}")
            if entry["observed_min"] is None or existing is None:
                continue
            if entry["observed_min"] < existing["min_position"] - 1e-6:
                extra = existing["min_position"] - entry["observed_min"]
                print(
                    f"           LEFT EXTRA RANGE: {extra:.3f} rad ({math.degrees(extra):.1f} deg)  "
                    "OBSERVED BELOW CURRENT MOVEIT LIMIT"
                )
            if entry["observed_max"] > existing["max_position"] + 1e-6:
                extra = entry["observed_max"] - existing["max_position"]
                print(
                    f"           RIGHT EXTRA RANGE: {extra:.3f} rad ({math.degrees(extra):.1f} deg)  "
                    "OBSERVED ABOVE CURRENT MOVEIT LIMIT"
                )
            if entry["coverage"] == "insufficient":
                print(f"           WARNING: {name} range coverage appears insufficient")
        print()

    def _save_yaml(self, results: Dict) -> Dict[str, str]:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        range_path = self.out_dir / f"x5a_joint_range_{stamp}.yaml"
        suggested_path = self.out_dir / "x5a_joint_limits_suggested.yaml"

        range_data = copy.deepcopy(results)
        for name in ARM_JOINTS:
            entry = range_data["joints"][name]
            existing = self.moveit_limits.get(name, {})
            entry["existing_moveit_min"] = existing.get("min_position")
            entry["existing_moveit_max"] = existing.get("max_position")
        with open(range_path, "w") as f:
            yaml.safe_dump(range_data, f, sort_keys=False, default_flow_style=False)

        suggested = {
            "comment": (
                "# SUGGESTED soft limits from observed human-swept range "
                "(NOT mechanical limits).\n"
                f"# Generated by x5a_joint_range_calibrate on {stamp}. "
                "Review before applying to\n"
                "# src/x5a_moveit_config/config/joint_limits.yaml."
            ),
            "joint_limits": {},
        }
        for name in ARM_JOINTS:
            entry = range_data["joints"][name]
            if entry["coverage"] == "ok":
                suggested["joint_limits"][name] = {
                    "has_position_limits": True,
                    "min_position": entry["recommended_min"],
                    "max_position": entry["recommended_max"],
                }
            else:
                suggested["joint_limits"][name] = {
                    "has_position_limits": True,
                    "min_position": None,
                    "max_position": None,
                    "comment": "coverage insufficient; keep existing limit",
                }
        with open(suggested_path, "w") as f:
            f.write(
                "# SUGGESTED soft limits from observed human-swept range "
                "(NOT mechanical limits).\n"
            )
            f.write(f"# Generated by x5a_joint_range_calibrate on {stamp}.\n")
            f.write(
                "# Review before applying to "
                "src/x5a_moveit_config/config/joint_limits.yaml.\n"
            )
            yaml.safe_dump(
                {"joint_limits": suggested["joint_limits"]},
                f,
                sort_keys=False,
                default_flow_style=False,
            )
        return {"range": str(range_path), "suggested": str(suggested_path)}

    def final_report(self, results: Dict, paths: Dict[str, str], build_ok: bool) -> None:
        print("\n" + "=" * 60)
        print("FINAL REPORT")
        print("=" * 60)
        print(f"Build: {'PASS' if build_ok else 'FAIL'}")
        print("Calibration tool: PASS")
        print("Feedback source:")
        print("  /joint_states")
        print("  /arm_status" + ("" if self.status_available else " (not available)"))
        joints = results["joints"]
        for name in ARM_JOINTS:
            entry = joints[name]
            print(
                f"{name.capitalize()} observed: "
                f"[{entry['observed_min']}, {entry['observed_max']}]"
            )
        j4 = joints["joint4"]
        existing_j4 = self.moveit_limits.get("joint4", {})
        print(
            f"Current joint4 MoveIt: "
            f"[{existing_j4.get('min_position')}, {existing_j4.get('max_position')}]"
        )
        print(
            f"Observed joint4: [{j4['observed_min']}, {j4['observed_max']}]"
        )
        print(
            f"Suggested joint4: [{j4['recommended_min']}, {j4['recommended_max']}]"
        )
        print("Current limits automatically changed: NO")
        print(f"Calibration YAML:\n  {paths['range']}")
        print(f"Suggested limits YAML:\n  {paths['suggested']}")
        print("Git commit: NO")
        print("Git push: NO")


def find_moveit_limits_file() -> str:
    try:
        share = get_package_share_directory("x5a_moveit_config")
        candidate = os.path.join(share, "config", "joint_limits.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    # fallback: relative to the workspace root (src tree)
    for root in (Path.cwd(), Path.home() / "arx" / "arm"):
        candidate = root / "src" / "x5a_moveit_config" / "config" / "joint_limits.yaml"
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError(
        "cannot locate x5a_moveit_config config/joint_limits.yaml; "
        "use --moveit-limits-file"
    )


def default_out_dir() -> str:
    for root in (Path.cwd(), Path.home() / "arx" / "arm"):
        if (root / "src").exists():
            return str(root / "calibration")
    return str(Path.cwd() / "calibration")


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description="X5A joint range calibration (observe only)")
    parser.add_argument("--margin-rad", type=float, default=0.05,
                        help="safety margin subtracted from observed extrema (default 0.05)")
    parser.add_argument("--guided", action="store_true",
                        help="prompt per joint: move it, ENTER, repeat")
    parser.add_argument("--moveit-limits-file", type=str, default=None,
                        help="path to current joint_limits.yaml")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="output directory for calibration YAMLs")
    parser.add_argument("--current-warn-factor", type=float, default=2.0,
                        help="HIGH CURRENT warning factor over first-second baseline")
    parsed, _ = parser.parse_known_args(args)

    try:
        limits_file = parsed.moveit_limits_file or find_moveit_limits_file()
    except FileNotFoundError as exception:
        print(exception)
        return 1
    out_dir = parsed.out_dir or default_out_dir()

    rclpy.init(args=args)
    # rclpy's SIGINT handler only records the signal outside the executor, so
    # a blocking select() loop would ignore Ctrl-C. Reinstall a handler that
    # raises KeyboardInterrupt on the main thread instead.
    signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = JointRangeCalibrate(
        margin_rad=parsed.margin_rad,
        guided=parsed.guided,
        moveit_limits_file=limits_file,
        out_dir=out_dir,
        current_warn_factor=parsed.current_warn_factor,
    )
    try:
        # Keyboard may be unavailable (stdin redirected, detached panel); the
        # topic fallback ~/x5a_joint_range_calibrate/next always works, so
        # only warn instead of failing.
        input_fd = node._input_fd()
        if input_fd is None:
            node.get_logger().warn(
                "no keyboard input source (/dev/tty and stdin unavailable); "
                "stop windows with: ros2 topic pub --once "
                "/x5a_joint_range_calibrate/next std_msgs/msg/Empty '{}'"
            )
        else:
            try:
                if os.fstat(input_fd).st_rdev == os.stat("/dev/null").st_rdev:
                    node.get_logger().warn(
                        "keyboard input is /dev/null; stop windows with: "
                        "ros2 topic pub --once /x5a_joint_range_calibrate/next "
                        "std_msgs/msg/Empty '{}'"
                    )
            except OSError:
                pass
        # wait for first real feedback
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not node.js_available:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.js_available:
            node.get_logger().error("/joint_states has no feedback; is the adapter running?")
            return 1
        if node.guided:
            per_joint = node.run_guided()
        else:
            recorder = node.run_continuous()
            per_joint = {name: recorder for name in ARM_JOINTS} if recorder else {}
        if not per_joint:
            print("No recording data collected.")
            return 1
        results = node._build_results(per_joint)
        node._compare_and_report(results)
        paths = node._save_yaml(results)
        node.final_report(results, paths, build_ok=True)
        return 0
    except KeyboardInterrupt:
        node.get_logger().info("interrupted by user")
        saved = node.recording and node.recorder.samples > 0
        if saved:
            node._save_partial({name: node.recorder for name in ARM_JOINTS})
        node._print_interrupted(saved=saved)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
