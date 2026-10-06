"""MAVROS 读写适配层（ROS1）。

替代 ROS2 版的 unification_adapter.py：
- ROS2 通过 Micro XRCE-DDS + px4_msgs 直接收发 uORB；
- ROS1 通过 MAVROS（MAVLink）话题与服务交互，MAVROS 已替我们完成 NED<->ENU 转换，
  因此本文件不再包含坐标变换，也不再需要轮询 VehicleCommandAck（服务调用同步返回成败）。

对外输出的快照仍为 ENU（位置/速度）、姿态为 FLU 四元数，与 ROS2 版保持一致。
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from threading import Lock
from typing import Optional

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, TwistStamped, Vector3
from mavros_msgs.msg import AttitudeTarget, BatteryStatus, ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from sensor_msgs.msg import Imu

from .models import DroneSnapshot, TaskCommand, TaskType

# MAV_STATE（mavros_msgs/State.system_status）
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
MAV_STATE_CRITICAL = 5
MAV_STATE_EMERGENCY = 6
MAV_STATE_FLIGHT_TERMINATION = 8

# MAVLink 命令
MAV_CMD_COMPONENT_ARM_DISARM = 400
ARMING_MAGIC_KILL = 21196.0

# PositionTarget type_mask 位
IGNORE_PX = 1
IGNORE_PY = 2
IGNORE_PZ = 4
IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
IGNORE_YAW = 1024
IGNORE_YAW_RATE = 2048

# AttitudeTarget type_mask 位
IGNORE_ROLL_RATE = 1
IGNORE_PITCH_RATE = 2
IGNORE_YAW_RATE = 4
IGNORE_THRUST = 64
IGNORE_ATTITUDE = 128


def yaw_to_quat_enu(yaw: float) -> tuple[float, float, float, float]:
    """ENU/FLU 下仅偏航 -> 四元数 (w,x,y,z)。"""
    half = 0.5 * float(yaw)
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def quat_wxyz_to_xyzw(q):
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])


class MavrosAdapter:
    """封装 MAVROS 状态订阅与控制发布。"""

    def __init__(self, namespace: str = "mavros", frame_id: str = "map") -> None:
        self.ns = namespace.strip("/")
        self.frame_id = frame_id
        self._lock = Lock()
        self._snapshot = DroneSnapshot()
        self._topic_rx_ms: dict[str, int] = {}
        self._hover_setpoint: Optional[tuple[float, float, float, float]] = None
        self._services_ready = False

        # ---- 订阅（MAVROS 输出，均为 ENU/FLU）----
        rospy.Subscriber(f"{namespace}/state", State, self._on_state)
        rospy.Subscriber(f"{namespace}/local_position/pose", PoseStamped, self._on_pose)
        rospy.Subscriber(
            f"{namespace}/local_position/velocity_local", TwistStamped, self._on_velocity
        )
        rospy.Subscriber(f"{namespace}/imu/data", Imu, self._on_imu)
        rospy.Subscriber(f"{namespace}/battery", BatteryStatus, self._on_battery)
        rospy.Subscriber(f"{namespace}/extended_state", ExtendedState, self._on_extended_state)
        rospy.Subscriber(f"{namespace}/home_pose", PoseStamped, self._on_home_pose)

        # ---- 发布（设定值）----
        self.pub_position = rospy.Publisher(
            f"{namespace}/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.pub_raw_local = rospy.Publisher(
            f"{namespace}/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.pub_raw_attitude = rospy.Publisher(
            f"{namespace}/setpoint_raw/attitude", AttitudeTarget, queue_size=10
        )

        # ---- 服务代理（调用前 ensure_services）----
        self.cli_arming = rospy.ServiceProxy(f"{namespace}/cmd/arming", CommandBool)
        self.cli_set_mode = rospy.ServiceProxy(f"{namespace}/set_mode", SetMode)
        self.cli_command = rospy.ServiceProxy(f"{namespace}/cmd/command", CommandLong)

    # ------------------------------------------------------------------
    # 服务与快照
    # ------------------------------------------------------------------
    def ensure_services(self, timeout: float = 15.0) -> None:
        """等待 MAVROS 服务可用（仅执行一次）。"""
        if self._services_ready:
            return
        for svc in (f"{self.ns}/cmd/arming", f"{self.ns}/set_mode", f"{self.ns}/cmd/command"):
            rospy.wait_for_service(svc, timeout=timeout)
        self._services_ready = True

    def _topic(self, name: str) -> None:
        self._topic_rx_ms[name] = int(time.time() * 1000)

    def get_snapshot(self) -> DroneSnapshot:
        with self._lock:
            # 浅拷贝，避免主循环读取期间被回调修改
            return replace(self._snapshot)

    def telemetry_debug(self, now_ms: int) -> dict:
        with self._lock:
            ages = {k: (now_ms - ts if ts else None) for k, ts in self._topic_rx_ms.items()}
        stale = [k for k, a in ages.items() if a is None or a > 2000]
        hint = None
        if ages.get("state") is None:
            hint = "未收到 mavros/state：确认 MAVROS 已启动且 fcu_url 指向 PX4（udp://:14540）"
        elif stale:
            hint = f"下列话题超过 2s 无更新: {', '.join(stale)}"
        return {"client_active": True, "topic_age_ms": ages, "hint": hint}

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_mode(mode: str) -> str:
        m = (mode or "").strip().upper()
        return m.replace(".", "_")

    def _on_state(self, msg: State) -> None:
        self._topic("state")
        failsafe = msg.system_status in (
            MAV_STATE_CRITICAL,
            MAV_STATE_EMERGENCY,
            MAV_STATE_FLIGHT_TERMINATION,
        )
        with self._lock:
            s = self._snapshot
            s.connected = bool(msg.connected)
            s.armed = bool(msg.armed)
            s.mode = self._normalize_mode(msg.mode)
            s.failsafe = failsafe
            if failsafe:
                s.failsafe_reason = f"system_status={msg.system_status}"
            else:
                s.failsafe_reason = ""
            s.updated_at_ms = int(time.time() * 1000)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._topic("pose")
        p = msg.pose.position
        with self._lock:
            self._snapshot.position = (float(p.x), float(p.y), float(p.z))
            self._snapshot.local_position_xy_valid = True
            self._snapshot.local_position_z_valid = True
            self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_velocity(self, msg: TwistStamped) -> None:
        self._topic("velocity")
        v = msg.twist.linear
        with self._lock:
            self._snapshot.velocity = (float(v.x), float(v.y), float(v.z))
            self._snapshot.local_velocity_xy_valid = True
            self._snapshot.local_velocity_z_valid = True
            self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_imu(self, msg: Imu) -> None:
        self._topic("imu")
        q = msg.orientation
        with self._lock:
            self._snapshot.attitude_quat = (
                float(q.w),
                float(q.x),
                float(q.y),
                float(q.z),
            )
            self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_battery(self, msg: BatteryStatus) -> None:
        self._topic("battery")
        with self._lock:
            self._snapshot.battery_remaining = float(msg.percentage)
            self._snapshot.battery_voltage = float(msg.voltage)
            self._snapshot.updated_at_ms = int(time.time() * 1000)

    def _on_extended_state(self, msg: ExtendedState) -> None:
        self._topic("extended_state")
        # landed_state: 0 UNDEFINED, 1 ON_GROUND, 2 IN_AIR, 3 TAKEOFF, 4 LANDING
        with self._lock:
            self._snapshot.connected = True

    def _on_home_pose(self, msg: PoseStamped) -> None:
        self._topic("home_pose")
        p = msg.pose.position
        with self._lock:
            s = self._snapshot
            s.home_position = (float(p.x), float(p.y), float(p.z))
            s.home_valid = True
            s.home_updated_at_ms = int(time.time() * 1000)

    # ------------------------------------------------------------------
    # 模式 / 解锁（服务，同步返回）
    # ------------------------------------------------------------------
    @staticmethod
    def _to_mavros_custom_mode(internal_mode: str) -> str:
        m = internal_mode.strip().upper()
        mapping = {
            "OFFBOARD": "OFFBOARD",
            "POSCTL": "POSCTL",
            "POSITION": "POSCTL",
            "LAND": "AUTO.LAND",
            "HOLD": "AUTO.LOITER",
            "RETURN_HOME": "AUTO.RTL",
            "MANUAL": "MANUAL",
            "ALTCTL": "ALTCTL",
            "ALTITUDE": "ALTCTL",
            "AUTO": "AUTO.MISSION",
            "ACRO": "ACRO",
            "STABILIZED": "STABILIZED",
            "STAB": "STABILIZED",
            "RATTITUDE": "RATTITUDE",
        }
        return mapping.get(m, m)

    def set_mode(self, internal_mode: str) -> tuple[bool, str]:
        """切主模式（内部模式名）。同步返回 (success, detail)。"""
        self.ensure_services()
        custom = self._to_mavros_custom_mode(internal_mode)
        resp = self.cli_set_mode.call(base_mode=0, custom_mode=custom)
        if resp.mode_sent:
            return True, f"MODE {custom}"
        return False, f"set_mode({custom}) 被拒绝"

    def arm(self, value: bool) -> tuple[bool, str]:
        """解锁/上锁（同步）。"""
        self.ensure_services()
        resp = self.cli_arming.call(value=bool(value))
        if resp.success:
            return True, "ARMED" if value else "DISARMED"
        return False, f"arming({value}) 被拒绝: {resp.result}"

    def kill_switch(self) -> tuple[bool, str]:
        """紧急停桨：COMPONENT_ARM_DISARM + magic 21196。"""
        self.ensure_services()
        resp = self.cli_command.call(
            broadcast=False,
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0.0,
            param2=ARMING_MAGIC_KILL,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )
        return bool(resp.success), f"KILL result={resp.result}"

    # ------------------------------------------------------------------
    # 模式切换并确认（轮询快照）
    # ------------------------------------------------------------------
    def current_mode(self) -> str:
        with self._lock:
            return self._snapshot.mode

    def is_mode(self, name: str) -> bool:
        return self.current_mode() == name.upper()

    def wait_mode(self, expected: str, timeout_ms: int = 8000) -> tuple[bool, str]:
        expected = expected.upper()
        start = int(time.time() * 1000)
        while int(time.time() * 1000) - start <= timeout_ms:
            if self.current_mode() == expected:
                return True, "OK"
            time.sleep(0.02)
        return False, f"模式未在 {timeout_ms}ms 内变为 {expected}，当前={self.current_mode()}"

    def _prime_position_stream(self, duration_s: float = 1.0, rate_hz: float = 20.0) -> None:
        """切 OFFBOARD 前先流式发布当前位置（MAVLink 要求 offboard 链路已建立）。"""
        snap = self.get_snapshot()
        px, py, pz = snap.position
        qw, qx, qy, qz = snap.attitude_quat
        yaw = _yaw_from_quat_enu((qw, qx, qy, qz))
        deadline = time.time() + duration_s
        dt = 1.0 / rate_hz
        while time.time() < deadline:
            self._publish_position(px, py, pz, yaw)
            time.sleep(dt)

    def switch_to_offboard_and_confirm(self) -> None:
        if self.is_mode("OFFBOARD"):
            return
        snap = self.get_snapshot()
        if not snap.connected:
            raise RuntimeError("飞控未连接，无法切入 OFFBOARD")
        self._prime_position_stream()
        ok, detail = self.set_mode("OFFBOARD")
        if not ok:
            raise RuntimeError(f"切入 OFFBOARD {detail}")
        nav_ok, nav_detail = self.wait_mode("OFFBOARD", timeout_ms=10000)
        if not nav_ok:
            raise RuntimeError(f"切入 OFFBOARD 后状态未就绪: {nav_detail}")

    def switch_to_posctl_and_confirm(self) -> None:
        if self.is_mode("POSCTL"):
            return
        ok, detail = self.set_mode("POSCTL")
        if not ok:
            raise RuntimeError(f"切入 POSCTL {detail}")
        nav_ok, nav_detail = self.wait_mode("POSCTL", timeout_ms=8000)
        if not nav_ok:
            raise RuntimeError(f"切入 POSCTL 后状态未就绪: {nav_detail}")

    # ------------------------------------------------------------------
    # 设定值发布（输入 ENU/FLU）
    # ------------------------------------------------------------------
    def _stamp_header(self):
        h = rospy.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self.frame_id
        return h

    def _publish_position(self, x: float, y: float, z: float, yaw: float) -> None:
        q = yaw_to_quat_enu(yaw)
        msg = PoseStamped()
        msg.header = self._stamp_header()
        msg.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=quat_wxyz_to_xyzw(q),
        )
        self.pub_position.publish(msg)

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        """速度 + 期望偏航：用 setpoint_raw/local（PositionTarget）。"""
        msg = PositionTarget()
        msg.header = self._stamp_header()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # 忽略位置、加速度、偏航速率；保留速度与偏航
        msg.type_mask = (
            IGNORE_PX | IGNORE_PY | IGNORE_PZ
            | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
            | IGNORE_YAW_RATE
        )
        msg.velocity = Vector3(x=float(vx), y=float(vy), z=float(vz))
        msg.yaw = _yaw_enu_to_mavros(yaw)
        self.pub_raw_local.publish(msg)

    def _publish_attitude(
        self,
        q_wxyz,
        thrust: float,
        *,
        body_rate=None,
        use_attitude: bool = True,
    ) -> None:
        """姿态 + 推力（setpoint_raw/attitude）。body_rate 非空时走角速度模式。"""
        msg = AttitudeTarget()
        msg.header = self._stamp_header()
        if use_attitude:
            q_enu = q_wxyz
            msg.orientation = quat_wxyz_to_xyzw(q_enu)
            msg.type_mask = IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE
        else:
            msg.type_mask = IGNORE_ATTITUDE
            if body_rate is not None:
                msg.body_rate = Vector3(
                    x=float(body_rate[0]), y=float(body_rate[1]), z=float(body_rate[2])
                )
        msg.thrust = float(thrust)
        self.pub_raw_attitude.publish(msg)

    # ------------------------------------------------------------------
    # 复合实时设定（realtime 通道）
    # ------------------------------------------------------------------
    def publish_composite_setpoint(self, **kw) -> None:
        use_position = kw.get("use_position", False)
        use_velocity = kw.get("use_velocity", False)
        use_attitude = kw.get("use_attitude", False)
        use_acceleration = kw.get("use_acceleration", False)
        use_yaw_rate = kw.get("use_yaw_rate", False)

        if use_attitude:
            q = (kw.get("qw", 1.0), kw.get("qx", 0.0), kw.get("qy", 0.0), kw.get("qz", 0.0))
            thrust = float(kw.get("thrust_z", 0.5))
            self._publish_attitude(q, thrust, use_attitude=True)
            return

        if use_position:
            self._publish_position(
                kw.get("x", 0.0), kw.get("y", 0.0), kw.get("z", 0.0), kw.get("yaw", 0.0)
            )
            return

        if use_velocity or use_acceleration:
            self._publish_velocity(
                kw.get("vx", 0.0), kw.get("vy", 0.0), kw.get("vz", 0.0), kw.get("yaw", 0.0)
            )
            return

        if use_yaw_rate:
            # 角速度 + 推力模式
            rate = (0.0, 0.0, float(kw.get("yaw_rate", 0.0)))
            self._publish_attitude(
                (1.0, 0.0, 0.0, 0.0),
                float(kw.get("thrust_z", 0.5)),
                body_rate=rate,
                use_attitude=False,
            )

    # ------------------------------------------------------------------
    # 悬停与返航
    # ------------------------------------------------------------------
    def clear_hover(self) -> None:
        self._hover_setpoint = None

    def publish_hover(self, *, recapture: bool = False) -> None:
        """锁存当前 ENU 位姿并持续发布，避免跟随噪声漂移。"""
        if recapture or self._hover_setpoint is None:
            snap = self.get_snapshot()
            px, py, pz = snap.position
            qw, qx, qy, qz = snap.attitude_quat
            yaw = _yaw_from_quat_enu((qw, qx, qy, qz))
            self._hover_setpoint = (px, py, pz, yaw)
        x, y, z, yaw = self._hover_setpoint
        self._publish_position(x, y, z, yaw)

    def return_home(self) -> tuple[bool, str]:
        """PX4 内置返航 + 降落：切 AUTO.RTL。"""
        return self.set_mode("RETURN_HOME")

    def capture_home_at_current_position(self) -> bool:
        """解锁时将当前本地位置记录为内部 home（用于显示；PX4 自身也会设 home）。"""
        snap = self.get_snapshot()
        if not (snap.local_position_xy_valid and snap.local_position_z_valid):
            return False
        now_ms = int(time.time() * 1000)
        with self._lock:
            s = self._snapshot
            s.home_position = (
                float(snap.position[0]),
                float(snap.position[1]),
                float(snap.position[2]),
            )
            s.home_valid = True
            s.home_updated_at_ms = now_ms
            s.updated_at_ms = now_ms
        return True

    def clear_home_on_disarm(self) -> None:
        """上锁后清除内部 home。"""
        with self._lock:
            s = self._snapshot
            s.home_valid = False
            s.home_position = (0.0, 0.0, 0.0)
            s.home_updated_at_ms = 0

    # ------------------------------------------------------------------
    # 任务入口（供业务层调用）
    # ------------------------------------------------------------------
    def publish_task(self, task: TaskCommand) -> None:
        p = task.payload
        t = task.task_type
        if t == TaskType.POSITION_CONTROL:
            self._publish_position(
                float(p.get("x", 0.0)),
                float(p.get("y", 0.0)),
                float(p.get("z", 0.0)),
                float(p.get("yaw", 0.0)),
            )
        elif t == TaskType.VELOCITY_CONTROL:
            self._publish_velocity(
                float(p.get("vx", 0.0)),
                float(p.get("vy", 0.0)),
                float(p.get("vz", 0.0)),
                float(p.get("yaw", 0.0)),
            )
        elif t == TaskType.ARMING:
            ok, detail = self.arm(bool(p.get("arm", False)))
            if not ok:
                raise RuntimeError(detail)
        elif t == TaskType.MODE_SWITCH:
            ok, detail = self.set_mode(str(p.get("mode", "")))
            if not ok:
                raise RuntimeError(detail)
        elif t == TaskType.KILL_SWITCH:
            ok, detail = self.kill_switch()
            if not ok:
                raise RuntimeError(detail)
        else:
            raise ValueError(f"不支持的任务类型: {t}")

    def last_command_sent(self) -> str:
        # ROS1 服务同步返回，ACK 由调用结果体现；保留接口占位。
        return "mavros-service"


def _yaw_from_quat_enu(q) -> float:
    """ENU/FLU 四元数 -> 偏航。"""
    qw, qx, qy, qz = q
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_enu_to_mavros(yaw: float) -> float:
    """MAVROS PositionTarget.yaw 同样按 ENU 输入（MAVROS 内部转 NED），直接返回。"""
    return float(yaw)
