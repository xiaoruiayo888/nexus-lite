"""视觉里程计反向桥（ROS1）。

方向：Gazebo 里程计（nav_msgs/Odometry，ENU）-> MAVROS 视觉定位输入。
- output_type=pose：发布 geometry_msgs/PoseStamped 到 mavros/vision_pose/pose（最常用）
- output_type=odom：发布 nav_msgs/Odometry 到 mavros/odometry/in

支持位置/姿态偏移修正，并可按固定频率重发布（MAVROS 视觉定位需要持续流）。
"""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def rpy_to_matrix(rpy) -> np.ndarray:
    cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_quat(R) -> tuple[float, float, float, float]:
    """旋转矩阵 -> (w,x,y,z)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def q_mult(q1, q0) -> tuple[float, float, float, float]:
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return (
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    )


class OdomBridge:
    """订阅 Gazebo 里程计，转换后发布给 MAVROS。"""

    def __init__(
        self,
        *,
        input_topic: str,
        input_type: str = "odometry",
        output_topic: str = "mavros/vision_pose/pose",
        output_type: str = "pose",
        output_frame: str = "map",
        position_offset=(0.0, 0.0, 0.0),
        rotation_offset_rpy=(0.0, 0.0, 0.0),
        quality: int = 90,
        publish_rate_hz: float = 30.0,
    ) -> None:
        self.output_type = output_type.strip().lower()
        self.output_frame = output_frame
        self._offset_p = np.asarray(position_offset, dtype=float)
        self._offset_R = rpy_to_matrix(np.asarray(rotation_offset_rpy, dtype=float))
        self._offset_q = matrix_to_quat(self._offset_R)
        self._quality = int(quality)
        self._lock = threading.Lock()
        self._latest: Optional[dict] = None
        self._timer = None

        msg_type = Odometry if input_type.strip().lower() == "odometry" else PoseStamped
        rospy.Subscriber(input_topic, msg_type, self._on_input, queue_size=20)

        if self.output_type == "odom":
            self.pub = rospy.Publisher(output_topic, Odometry, queue_size=20)
        else:
            self.pub = rospy.Publisher(output_topic, PoseStamped, queue_size=20)

        rate = float(publish_rate_hz)
        if rate > 0.0:
            self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._on_timer)

    @classmethod
    def from_rosparams(cls) -> "OdomBridge":
        g = lambda k, d: rospy.get_param(f"~{k}", d)
        return cls(
            input_topic=g("input_topic", "/drone260/odom"),
            input_type=g("input_type", "odometry"),
            output_topic=g("output_topic", "mavros/vision_pose/pose"),
            output_type=g("output_type", "pose"),
            output_frame=g("output_frame", "map"),
            position_offset=g("position_offset", [0.0, 0.0, 0.0]),
            rotation_offset_rpy=g("rotation_offset_rpy", [0.0, 0.0, 0.0]),
            quality=g("quality", 90),
            publish_rate_hz=g("publish_rate_hz", 30.0),
        )

    def start(self) -> None:
        rospy.spin()

    # ------------------------------------------------------------------
    def _on_input(self, msg) -> None:
        if isinstance(msg, Odometry):
            pose = msg.pose.pose
            stamp = msg.header.stamp
            frame = msg.header.frame_id
        else:
            pose = msg.pose
            stamp = msg.header.stamp
            frame = msg.header.frame_id

        p = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        q = (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )

        p = self._offset_R @ p + self._offset_p
        q = q_mult(self._offset_q, q)

        out = {"p": p, "q": q, "stamp": stamp, "frame": frame}
        with self._lock:
            self._latest = out

        if self._timer is None:
            self._publish(out)

    def _on_timer(self, _event) -> None:
        with self._lock:
            out = self._latest
        if out is not None:
            self._publish(out)

    def _publish(self, out) -> None:
        p, q = out["p"], out["q"]
        stamp = out["stamp"] if out["stamp"].to_sec() > 0 else rospy.Time.now()

        if self.output_type == "odom":
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self.output_frame
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (
                float(p[0]),
                float(p[1]),
                float(p[2]),
            )
            (
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ) = q
            # 用协方差表达质量：quality 越高协方差越小
            var = max(1e-4, (100.0 - self._quality) * 1e-3)
            for i in range(6):
                msg.pose.covariance[i * 6 + i] = var
            self.pub.publish(msg)
        else:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.output_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
                float(p[0]),
                float(p[1]),
                float(p[2]),
            )
            (
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ) = q
            self.pub.publish(msg)
