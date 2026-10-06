#!/usr/bin/env python3
"""ROS1 示例公共客户端：封装任务/实时话题发布与状态订阅（rospy）。

对外话题与 ROS2 版完全一致：
- 发布 /px4_bridge/in/task_cmd、/px4_bridge/in/realtime_control
- 订阅 /px4_bridge/out/status
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rospy
from px4_bridge_msgs.msg import BridgeStatus, RealtimeControl, TaskCommand


class _Logger:
    def info(self, msg):
        rospy.loginfo(msg)

    def warn(self, msg):
        rospy.logwarn(msg)

    def error(self, msg):
        rospy.logerr(msg)


class BridgeClient:
    """桥客户端。"""

    def __init__(self, node_name: str) -> None:
        rospy.init_node(node_name, anonymous=True)
        self._logger = _Logger()
        self._latest_status: Optional[BridgeStatus] = None

        self.pub_task = rospy.Publisher(
            "/px4_bridge/in/task_cmd", TaskCommand, queue_size=10
        )
        self.pub_rt = rospy.Publisher(
            "/px4_bridge/in/realtime_control", RealtimeControl, queue_size=10
        )
        rospy.Subscriber(
            "/px4_bridge/out/status", BridgeStatus, self._on_status, queue_size=10
        )

    def _on_status(self, msg: BridgeStatus) -> None:
        self._latest_status = msg

    def get_logger(self) -> _Logger:
        return self._logger

    # ------------------------------------------------------------------
    def submit_task(
        self,
        task_id: str,
        task_type: str,
        payload: dict,
        deadline_ms: int = 5000,
    ) -> None:
        msg = TaskCommand()
        msg.task_id = str(task_id)
        msg.task_type = str(task_type)
        msg.deadline_ms = int(deadline_ms)

        if task_type == "ARMING":
            msg.arm = bool(payload.get("arm", False))
            msg.confirm_current_mode = bool(payload.get("confirm_current_mode", False))
        elif task_type == "MODE_SWITCH":
            msg.mode = str(payload.get("mode", ""))
        elif task_type == "POSITION_CONTROL":
            msg.x = float(payload.get("x", 0.0))
            msg.y = float(payload.get("y", 0.0))
            msg.z = float(payload.get("z", 0.0))
            msg.yaw = float(payload.get("yaw", 0.0))
            msg.arrival_radius_m = float(payload.get("arrival_radius_m", 0.0))
            msg.arrival_dz_m = float(payload.get("arrival_dz_m", 0.0))
            msg.arrival_speed_mps = float(payload.get("arrival_speed_mps", 0.0))
        elif task_type == "VELOCITY_CONTROL":
            msg.vx = float(payload.get("vx", 0.0))
            msg.vy = float(payload.get("vy", 0.0))
            msg.vz = float(payload.get("vz", 0.0))
            msg.yaw = float(payload.get("yaw", 0.0))

        self.pub_task.publish(msg)

    def publish_realtime(
        self, *, enable: bool, has_command: bool, **kw
    ) -> None:
        msg = RealtimeControl()
        msg.enable = bool(enable)
        msg.has_command = bool(has_command)
        msg.use_position = bool(kw.get("use_position", False))
        msg.use_velocity = bool(kw.get("use_velocity", False))
        msg.use_attitude = bool(kw.get("use_attitude", False))
        msg.use_acceleration = bool(kw.get("use_acceleration", False))
        msg.use_jerk = bool(kw.get("use_jerk", False))
        msg.use_yaw_rate = bool(kw.get("use_yaw_rate", False))
        msg.x = float(kw.get("x", 0.0))
        msg.y = float(kw.get("y", 0.0))
        msg.z = float(kw.get("z", 0.0))
        msg.vx = float(kw.get("vx", 0.0))
        msg.vy = float(kw.get("vy", 0.0))
        msg.vz = float(kw.get("vz", 0.0))
        msg.ax = float(kw.get("ax", 0.0))
        msg.ay = float(kw.get("ay", 0.0))
        msg.az = float(kw.get("az", 0.0))
        msg.yaw_rate = float(kw.get("yaw_rate", 0.0))
        msg.yaw = float(kw.get("yaw", 0.0))
        msg.qw = float(kw.get("qw", 1.0))
        msg.qx = float(kw.get("qx", 0.0))
        msg.qy = float(kw.get("qy", 0.0))
        msg.qz = float(kw.get("qz", 0.0))
        msg.thrust_z = float(kw.get("thrust_z", 0.5))
        msg.timestamp_ms = int(time.time() * 1000)
        self.pub_rt.publish(msg)

    # ------------------------------------------------------------------
    def spin_brief(self, seconds: float) -> None:
        rospy.sleep(max(0.0, float(seconds)))

    def latest_status(self) -> Optional[BridgeStatus]:
        return self._latest_status

    def status(self) -> BridgeStatus:
        st = self._latest_status
        if st is None:
            raise RuntimeError("尚未收到 /px4_bridge/out/status")
        return st

    def wait_status(self, timeout_s: float = 5.0) -> BridgeStatus:
        end = time.time() + timeout_s
        while time.time() < end:
            if self._latest_status is not None:
                return self._latest_status
            rospy.sleep(0.02)
        raise TimeoutError("等待 status 超时")

    def wait_armed(self, expected: bool, timeout_s: float = 10.0) -> BridgeStatus:
        end = time.time() + timeout_s
        while time.time() < end:
            st = self._latest_status
            if st is not None and bool(st.armed) == bool(expected):
                return st
            rospy.sleep(0.05)
        raise TimeoutError(f"等待 armed={expected} 超时")

    def wait_task_idle(
        self, timeout_s: float, after_task_id: Optional[str] = None
    ) -> BridgeStatus:
        """等待桥侧回到非 EXECUTING（且 active_task_id 已越过 after_task_id）。"""
        end = time.time() + timeout_s
        seen = after_task_id is None
        while time.time() < end:
            st = self._latest_status
            if st is not None:
                if not seen and st.active_task_id == after_task_id:
                    seen = True
                if seen and st.lifecycle_state != "EXECUTING":
                    return st
            rospy.sleep(0.05)
        raise TimeoutError(f"等待任务空闲超时(after={after_task_id})")

    def wait_until_arrived(
        self,
        x: float,
        y: float,
        z: float,
        *,
        radius_m: float,
        dz_m: float,
        speed_mps: float,
        timeout_s: float,
    ) -> BridgeStatus:
        end = time.time() + timeout_s
        while time.time() < end:
            st = self._latest_status
            if st is not None:
                px, py, pz = st.position
                vx, vy, vz = st.velocity
                horiz = math.hypot(x - px, y - py)
                speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                if horiz <= radius_m and abs(z - pz) <= dz_m and speed <= speed_mps:
                    return st
            rospy.sleep(0.05)
        raise TimeoutError("等待到达超时")

    def destroy_node(self) -> None:
        pass


def make_client(node_name: str) -> BridgeClient:
    return BridgeClient(node_name)
