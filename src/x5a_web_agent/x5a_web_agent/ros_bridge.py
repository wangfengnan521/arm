"""ROS 2 Action client for structured pick-place tasks.

This process never publishes /arm_cmd, joint commands, or trajectories.
It only talks to /x5a/pick_place and reads /x5a/task_stage.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from x5a_task_interfaces.action import X5aPickPlace


FeedbackCallback = Callable[[str], None]
BUSY_MESSAGE = "机器人正在执行上一任务，请稍后。"
ALLOWED_COLORS = ("red", "white", "orange", "nearest", "farthest")
TERMINAL_STAGES = frozenset({"SUCCESS", "FAILED", "IDLE", "DRY_RUN"})


class RosBridge:
    def __init__(
        self,
        dry_run: bool = False,
        action_name: str = "/x5a/pick_place",
    ) -> None:
        self.dry_run = bool(dry_run)
        self.action_name = action_name
        self._lock = threading.Lock()
        self._busy = False
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._client: Optional[ActionClient] = None
        self._goal_handle = None
        self._rclpy_owned = False
        self._stage = "IDLE"
        self._server_busy = False
        self._stage_cb: Optional[FeedbackCallback] = None

    def set_stage_callback(self, callback: Optional[FeedbackCallback]) -> None:
        self._stage_cb = callback

    @property
    def current_stage(self) -> str:
        return self._stage

    def start(self) -> None:
        if self._node is not None:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_owned = True
        self._node = Node("x5a_web_agent")
        self._client = ActionClient(self._node, X5aPickPlace, self.action_name)
        self._node.create_subscription(
            String, "/x5a/task_stage", self._on_stage_topic, 10
        )
        self._node.create_subscription(
            Bool, "/x5a/task_busy", self._on_busy_topic, 10
        )
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            f"web agent ROS bridge ready action={self.action_name} dry_run={self.dry_run}"
        )

    def _on_stage_topic(self, msg: String) -> None:
        stage = str(msg.data or "").strip() or "IDLE"
        self._stage = stage
        callback = self._stage_cb
        if callback is not None:
            try:
                callback(stage)
            except Exception:
                pass

    def _on_busy_topic(self, msg: Bool) -> None:
        self._server_busy = bool(msg.data)
        if not msg.data and self._stage in TERMINAL_STAGES:
            callback = self._stage_cb
            if callback is not None:
                try:
                    callback(self._stage or "IDLE")
                except Exception:
                    pass

    def shutdown(self) -> None:
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        self._client = None
        if self._rclpy_owned and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    @property
    def busy(self) -> bool:
        with self._lock:
            local_busy = self._busy
        if self._stage in TERMINAL_STAGES and not self._server_busy:
            return False
        return local_busy

    def server_ready(self, timeout: float = 0.5) -> bool:
        if self.dry_run:
            return True
        if self._client is None:
            return False
        return bool(self._client.wait_for_server(timeout_sec=timeout))

    def cancel(self) -> bool:
        handle = self._goal_handle
        if handle is None:
            return False
        try:
            handle.cancel_goal_async()
            return True
        except Exception:
            return False

    def send_pick_place(
        self,
        target_color: str,
        on_feedback: Optional[FeedbackCallback] = None,
        timeout: float = 180.0,
        skip_return_home: bool = False,
        reuse_frozen_box: bool = False,
    ) -> Dict[str, Any]:
        color = str(target_color or "").strip().lower()
        if color not in ALLOWED_COLORS:
            return {
                "success": False,
                "message": f"invalid target_color: {target_color!r}",
                "stage": "FAILED",
            }
        if self.dry_run:
            if on_feedback is not None:
                on_feedback("DRY_RUN")
            return {
                "success": True,
                "message": "DRY_RUN: 已解析任务，未发送 ROS Action",
                "stage": "DRY_RUN",
            }

        with self._lock:
            if self._busy:
                return {
                    "success": False,
                    "message": BUSY_MESSAGE,
                    "stage": "BUSY",
                }
            self._busy = True

        try:
            return self._send_locked(
                color,
                on_feedback,
                timeout,
                skip_return_home=skip_return_home,
                reuse_frozen_box=reuse_frozen_box,
            )
        finally:
            with self._lock:
                self._busy = False
                self._goal_handle = None

    def _send_locked(
        self,
        color: str,
        on_feedback: Optional[FeedbackCallback],
        timeout: float,
        skip_return_home: bool = False,
        reuse_frozen_box: bool = False,
    ) -> Dict[str, Any]:
        if self._client is None or self._node is None:
            return {
                "success": False,
                "message": "ROS bridge 未启动",
                "stage": "FAILED",
            }
        if not self._client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "message": "Task Server 未就绪（/x5a/pick_place）",
                "stage": "FAILED",
            }

        last_stage = "ACCEPTED"

        def _note(stage: str) -> None:
            nonlocal last_stage
            last_stage = stage
            if on_feedback is not None:
                on_feedback(stage)

        prev_cb = self._stage_cb

        def _topic_and_user(stage: str) -> None:
            _note(stage)
            if prev_cb is not None and prev_cb is not on_feedback:
                prev_cb(stage)

        self._stage_cb = _topic_and_user

        try:
            goal = X5aPickPlace.Goal()
            goal.target_color = color
            goal.skip_return_home = bool(skip_return_home)
            goal.reuse_frozen_box = bool(reuse_frozen_box)
            send_future = self._client.send_goal_async(goal)
            if not self._wait(send_future, 5.0):
                return {
                    "success": False,
                    "message": "发送 Action Goal 超时",
                    "stage": "FAILED",
                }
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return {
                    "success": False,
                    "message": BUSY_MESSAGE,
                    "stage": "BUSY",
                }
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            if not self._wait(
                result_future,
                timeout,
                extra_done=lambda: last_stage in TERMINAL_STAGES
                or self._stage in TERMINAL_STAGES,
            ):
                stage = last_stage if last_stage in TERMINAL_STAGES else self._stage
                if stage in TERMINAL_STAGES:
                    return {
                        "success": stage == "SUCCESS",
                        "message": "任务完成" if stage == "SUCCESS" else "任务失败",
                        "stage": stage,
                    }
                return {
                    "success": False,
                    "message": "等待 Task Server 结果超时",
                    "stage": stage or "FAILED",
                }
            wrapped = result_future.result()
            result = wrapped.result
            return {
                "success": bool(result.success),
                "message": result.message or ("任务完成" if result.success else "任务失败"),
                "stage": "SUCCESS" if result.success else (last_stage or "FAILED"),
            }
        finally:
            self._stage_cb = prev_cb

    @staticmethod
    def _wait(future, timeout: float, extra_done=None) -> bool:
        deadline = time.monotonic() + timeout
        seen_terminal = None
        while not future.done() and time.monotonic() < deadline:
            if extra_done is not None and extra_done():
                if seen_terminal is None:
                    seen_terminal = time.monotonic()
                elif time.monotonic() - seen_terminal >= 0.3:
                    return future.done()
            time.sleep(0.02)
        return future.done()
