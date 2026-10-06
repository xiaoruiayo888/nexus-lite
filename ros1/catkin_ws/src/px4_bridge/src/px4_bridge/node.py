"""精简主控制节点（ROS1 / rospy）：连接监控、流式设定点、实时控制、离散任务。

与 ROS2 版业务逻辑一致，区别：
- rospy 在后台线程执行订阅回调，主循环用 rospy.Rate 节拍，无需手动 spin；
- 解锁/切模式经 MAVROS 服务同步返回，不再轮询 ACK；
- 返航直接切 AUTO.RTL（PX4 内置），不再自行飞 home + LAND。
"""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import rospy

from .lifecycle import Lifecycle
from .mavros_adapter import MavrosAdapter
from .models import DroneSnapshot, LifecycleState, RuntimeContext, TaskCommand, TaskType
from .realtime_controller import RealtimeController
from .ros_topic_bridge import RosTopicBridge
from .task_server import TaskServerAdapter


class PX4BridgeNode:
    """控制桥（rospy）。"""

    _STREAM_TYPES = frozenset(
        {TaskType.POSITION_CONTROL, TaskType.VELOCITY_CONTROL}
    )

    def __init__(
        self,
        *,
        loop_hz: int = 20,
        connection_timeout_ms: int = 1000,
        realtime_timeout_ms: int = 300,
        realtime_max_speed_mps: float = 2.0,
        mav_namespace: str = "mavros",
    ) -> None:
        self.loop_hz = max(1, int(loop_hz))
        self.connection_timeout_ms = int(connection_timeout_ms)

        self.ctx = RuntimeContext()
        self.sm = Lifecycle(self.ctx)

        self.mav = MavrosAdapter(namespace=mav_namespace)
        self.task_adapter = TaskServerAdapter()
        self.realtime = RealtimeController(
            self.mav,
            timeout_ms=realtime_timeout_ms,
            max_speed_mps=realtime_max_speed_mps,
        )
        self.topic_bridge = RosTopicBridge(
            self.task_adapter, self.realtime, self.runtime_status
        )

        self._history: list[dict] = []
        self.stream_pending_task: Optional[TaskCommand] = None
        self.stream_pending_start_ms = 0
        self._last_armed_state: Optional[bool] = None

    # ------------------------------------------------------------------
    # 启停与主循环
    # ------------------------------------------------------------------
    def start(self) -> None:
        rospy.loginfo("节点启动")
        self.sm.transit(LifecycleState.WAITING_CONNECTION, "系统启动")
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def stop(self) -> None:
        rospy.loginfo("节点停止")

    def _tick(self) -> None:
        snapshot = self.mav.get_snapshot()
        if self._handle_armed_state_transitions(snapshot):
            self.topic_bridge.publish_status()
            return
        self._run_control_loop(snapshot)
        self.topic_bridge.publish_status()

    # ------------------------------------------------------------------
    def _run_control_loop(self, snapshot: DroneSnapshot) -> None:
        if self._process_connection(snapshot):
            return
        # 实时控制最高优先
        if self.realtime.enabled():
            if self.realtime.process_tick(
                self.sm,
                self.ctx,
                stream_abort=self._abort_stream,
                append_record=self._append_record,
                to_fault=self._to_fault,
                log_transition=self._log_transition,
            ):
                return
        # 新任务抢占当前流式任务
        if self.stream_pending_task is not None and self.task_adapter.pending_count() > 0:
            self._abort_stream("新任务抢占，中止当前流式任务", status="CANCELLED")
        if self.stream_pending_task is not None:
            self._tick_stream_pending(snapshot)
            return
        self._process_discrete(snapshot)

    def _process_connection(self, snapshot: DroneSnapshot) -> bool:
        if snapshot.failsafe:
            detail = snapshot.failsafe_reason.strip() or "原因未知"
            fault_msg = f"检测到 failsafe: {detail}"
            if self.sm.state != LifecycleState.FAULT or self.ctx.current_error != fault_msg:
                self._append_record("SYSTEM", "FAILED", fault_msg)
            self._to_fault(fault_msg)
            return True

        stale_ms = int(time.time() * 1000) - snapshot.updated_at_ms
        if snapshot.connected and stale_ms > self.connection_timeout_ms:
            self._append_record("SYSTEM", "FAILED", f"飞控连接超时: {stale_ms}ms")
            self._to_fault(f"飞控连接超时: {stale_ms}ms")
            return True

        if self.sm.state == LifecycleState.FAULT:
            pending = self.task_adapter.pending_count()
            recoverable = not snapshot.failsafe and (snapshot.connected or pending > 0)
            if recoverable:
                ok, from_s, to_s, reason = self.sm.transit(
                    LifecycleState.READY, "故障条件解除，自动恢复"
                )
                if ok:
                    self.ctx.current_error = None
                    self.ctx.active_task = None
                    self._log_transition(from_s.value, to_s.value, reason)
            else:
                return True

        if self.sm.state == LifecycleState.WAITING_CONNECTION and (
            snapshot.connected or self.task_adapter.pending_count() > 0
        ):
            ok, from_s, to_s, reason = self.sm.transit(
                LifecycleState.READY,
                "飞控连接可用" if snapshot.connected else "有待执行任务",
            )
            if ok:
                self._log_transition(from_s.value, to_s.value, reason)

        return False

    # ------------------------------------------------------------------
    # 离散任务
    # ------------------------------------------------------------------
    def _process_discrete(self, snapshot: DroneSnapshot) -> None:
        del snapshot
        if self.sm.state not in {LifecycleState.READY, LifecycleState.EXECUTING}:
            return
        task = self.task_adapter.fetch_task()
        if task is None:
            return
        if not self._enter_executing(task, f"接收任务: {task.task_id}"):
            return
        self._execute_task_payload(task)

    def _enter_executing(self, task: TaskCommand, reason: str) -> bool:
        ok, from_s, to_s, msg = self.sm.transit(LifecycleState.EXECUTING, reason)
        if not ok:
            rospy.logwarn(msg)
            self._append_record(task.task_id, "REJECTED", msg, task.task_type.value)
            return False
        self.ctx.active_task = task
        self._log_transition(from_s.value, to_s.value, msg)
        return True

    def _execute_task_payload(self, task: TaskCommand) -> None:
        try:
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                # ROS1：PX4 内置返航
                self._set_rth_phase("RTH_START", "切入 AUTO.RTL，由 PX4 返航降落")
                ok, detail = self.mav.return_home()
                if not ok:
                    raise RuntimeError(detail)
                self._set_rth_phase("RTH_DONE", "AUTO.RTL 已接受")
            elif task.task_type in self._STREAM_TYPES:
                self.mav.switch_to_offboard_and_confirm()
                self.mav.publish_task(task)
                self.stream_pending_task = task
                self.stream_pending_start_ms = int(time.time() * 1000)
                return
            else:
                if task.task_type == TaskType.ARMING:
                    self._ensure_posctl_before_arm_if_needed(task)
                # 服务同步执行，失败会抛异常
                self.mav.publish_task(task)
                if task.task_type == TaskType.ARMING and bool(
                    task.payload.get("arm", False)
                ):
                    self._capture_home_on_arm()

            ok, from_s, to_s, tr_reason = self.sm.transit(
                LifecycleState.READY, f"任务完成: {task.task_id}"
            )
            if ok:
                self._log_transition(from_s.value, to_s.value, tr_reason)
            self._append_record(task.task_id, "SUCCESS", "任务执行完成", task.task_type.value)
            self.ctx.active_task = None
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                self._set_rth_phase("IDLE", "")
        except Exception as exc:  # pylint: disable=broad-except
            self._append_record(task.task_id, "FAILED", str(exc), task.task_type.value)
            if (
                task.task_type == TaskType.MODE_SWITCH
                and str(task.payload.get("mode", "")).upper() == "RETURN_HOME"
            ):
                self._set_rth_phase("RTH_FAILED", str(exc))
            self._to_fault(f"任务执行失败: {task.task_id}, err={exc}")

    def _ensure_posctl_before_arm_if_needed(self, task: TaskCommand) -> None:
        if not bool(task.payload.get("arm", False)):
            return
        if task.payload.get("confirm_current_mode") is True:
            return
        snap = self.mav.get_snapshot()
        if not snap.connected:
            raise RuntimeError(
                "飞控未连接，拒绝解锁；若需在非定点模式下解锁请设置 payload.confirm_current_mode=true"
            )
        if self.mav.is_mode("POSCTL"):
            return
        self.mav.switch_to_posctl_and_confirm()

    # ------------------------------------------------------------------
    # 流式任务续发与到达判定
    # ------------------------------------------------------------------
    def _tick_stream_pending(self, snapshot: DroneSnapshot) -> None:
        task = self.stream_pending_task
        if task is None:
            return
        try:
            self.mav.publish_task(task)
        except Exception as exc:  # pylint: disable=broad-except
            self.stream_pending_task = None
            self._append_record(task.task_id, "FAILED", str(exc), task.task_type.value)
            self.ctx.active_task = None
            self._to_fault(f"流式任务发布失败: {task.task_id}, err={exc}")
            return

        now_ms = int(time.time() * 1000)
        elapsed = now_ms - self.stream_pending_start_ms

        if task.task_type == TaskType.POSITION_CONTROL:
            reached, detail = self._is_position_reached(task, snapshot)
            effective_deadline_ms = max(int(task.deadline_ms), 30_000)
            if reached:
                self._finish_stream(task, "SUCCESS", detail)
                return
            if elapsed >= effective_deadline_ms:
                tx = float(task.payload.get("x", 0.0))
                ty = float(task.payload.get("y", 0.0))
                tz = float(task.payload.get("z", 0.0))
                px, py, pz = snapshot.position
                horiz = math.hypot(tx - px, ty - py)
                dz = abs(tz - pz)
                self._finish_stream(
                    task,
                    "FAILED",
                    f"POSITION 到达超时({effective_deadline_ms}ms), Δxy={horiz:.2f}m Δz={dz:.2f}m",
                )
            return

        if task.task_type == TaskType.VELOCITY_CONTROL:
            if elapsed >= task.deadline_ms:
                self._finish_stream(
                    task, "SUCCESS", f"流式控制已持续 {task.deadline_ms}ms"
                )

    def _is_position_reached(
        self, task: TaskCommand, snapshot: DroneSnapshot
    ) -> tuple[bool, str]:
        tx = float(task.payload.get("x", 0.0))
        ty = float(task.payload.get("y", 0.0))
        tz = float(task.payload.get("z", 0.0))
        px, py, pz = snapshot.position
        vx, vy, vz = snapshot.velocity
        horiz = math.hypot(tx - px, ty - py)
        dz = abs(tz - pz)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        r_xy = float(task.payload.get("arrival_radius_m", 1.0))
        r_z = float(task.payload.get("arrival_dz_m", 0.6))
        speed_threshold = float(task.payload.get("arrival_speed_mps", 0.5))
        reached = horiz <= r_xy and dz <= r_z and speed <= speed_threshold
        detail = f"已到达 xy={horiz:.2f}m z={dz:.2f}m |v|={speed:.2f}m/s"
        return reached, detail

    def _finish_stream(self, task: TaskCommand, status: str, detail: str) -> None:
        self.stream_pending_task = None
        ok, from_s, to_s, reason = self.sm.transit(
            LifecycleState.READY, f"任务{status}: {task.task_id}"
        )
        if ok:
            self._log_transition(from_s.value, to_s.value, reason)
        self._append_record(task.task_id, status, detail, task.task_type.value)
        self.ctx.active_task = None
        if status == "SUCCESS":
            rospy.loginfo("任务完成: %s (%s)", task.task_id, detail)

    def _abort_stream(self, reason: str, status: str = "FAILED") -> None:
        task = self.stream_pending_task
        if task is None:
            return
        self.stream_pending_task = None
        self._append_record(task.task_id, status, reason, task.task_type.value)
        self.ctx.active_task = None
        ok, from_s, to_s, msg = self.sm.transit(LifecycleState.READY, "流式任务被中止")
        if ok:
            self._log_transition(from_s.value, to_s.value, msg)

    # ------------------------------------------------------------------
    def _to_fault(self, reason: str) -> None:
        self.stream_pending_task = None
        self.ctx.current_error = reason
        ok, from_s, to_s, tr_reason = self.sm.transit(LifecycleState.FAULT, reason)
        if ok:
            rospy.logerr("进入故障态: %s", reason)
            self._log_transition(from_s.value, to_s.value, tr_reason)

    def _log_transition(self, src: str, dst: str, reason: str) -> None:
        rospy.loginfo("状态转换: %s -> %s, reason=%s", src, dst, reason)

    def runtime_status(self) -> dict:
        snapshot = self.mav.get_snapshot()
        active_task_id = self.ctx.active_task.task_id if self.ctx.active_task else None
        now_ms = int(time.time() * 1000)
        return {
            "lifecycle_state": self.ctx.lifecycle_state.value,
            "active_task_id": active_task_id,
            "error": (
                self.ctx.current_error
                if self.sm.state == LifecycleState.FAULT
                else ""
            ),
            "rth_phase": self.ctx.rth_phase,
            "rth_detail": self.ctx.rth_detail,
            "task_history": [self._history[-1]] if self._history else [],
            "realtime": self.realtime.status(),
            "telemetry": self.mav.telemetry_debug(now_ms),
            "drone": {
                "connected": snapshot.connected,
                "mode": snapshot.mode,
                "armed": snapshot.armed,
                "position": list(snapshot.position),
                "velocity": list(snapshot.velocity),
                "attitude_quat": list(snapshot.attitude_quat),
                "battery_remaining": snapshot.battery_remaining,
                "battery_voltage": snapshot.battery_voltage,
                "failsafe": snapshot.failsafe,
                "failsafe_reason": snapshot.failsafe_reason,
                "home_valid": snapshot.home_valid,
                "home_position": list(snapshot.home_position),
                "updated_at_ms": snapshot.updated_at_ms,
            },
        }

    def _append_record(
        self, task_id: str, status: str, detail: str, task_type: str = "SYSTEM"
    ) -> None:
        self._history.append(
            {
                "task_id": task_id,
                "task_type": task_type,
                "status": status,
                "detail": detail,
                "timestamp_ms": int(time.time() * 1000),
            }
        )
        if len(self._history) > 50:
            self._history = self._history[-50:]

    def _set_rth_phase(self, phase: str, detail: str) -> None:
        self.ctx.rth_phase = phase
        self.ctx.rth_detail = detail

    def _capture_home_on_arm(self) -> None:
        if self.mav.capture_home_at_current_position():
            home = self.mav.get_snapshot().home_position
            rospy.loginfo("解锁时记录 home: x=%.2f y=%.2f z=%.2f", home[0], home[1], home[2])

    # ------------------------------------------------------------------
    def _handle_armed_state_transitions(self, snapshot: DroneSnapshot) -> bool:
        armed_now = bool(snapshot.armed)
        if self._last_armed_state is None:
            self._last_armed_state = armed_now
            return False
        armed_transition = not self._last_armed_state and armed_now
        disarmed_transition = self._last_armed_state and not armed_now
        self._last_armed_state = armed_now

        if armed_transition:
            self._capture_home_on_arm()

        if not disarmed_transition:
            return False

        self.mav.clear_home_on_disarm()
        rospy.loginfo("检测到上锁，清理任务")
        if self.realtime.enabled():
            self.realtime.disable()
        self.stream_pending_task = None
        self.ctx.active_task = None
        self.task_adapter.clear()
        if self.sm.state == LifecycleState.EXECUTING:
            ok, from_s, to_s, reason = self.sm.transit(LifecycleState.READY, "上锁清理")
            if ok:
                self._log_transition(from_s.value, to_s.value, reason)
        return True
