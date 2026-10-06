# px4_bridge 示例（ROS1 / Noetic / MAVROS）

本目录示例通过 **MAVROS（MAVLink）** 与 PX4 交互，演示 `px4_bridge` 的任务接口与实时控制接口。

## 前置

1. 已按 `ros1/README.md` 编译工作空间并 source：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
2. PX4 SITL（Gazebo Classic）已启动。
3. 启动 MAVROS + 控制桥：
   ```bash
   roslaunch px4_bridge mavros_bridge.launch
   ```
   （若 MAVROS 已单独启动，则 `roslaunch px4_bridge bridge.launch`。）

## 运行示例

新开终端，进入本目录后运行（示例用 `from common import ...`，需在本目录下执行）：

```bash
cd ~/catkin_ws/../examples   # 即 ros1/examples 实际路径
python3 01_arm_takeoff_land.py
```

| 脚本 | 内容 |
| --- | --- |
| `01_arm_takeoff_land.py` | 解锁 → 起飞 → 悬停 → 降落 → 上锁 |
| `02_position_point.py` | 指点飞行（POSITION_CONTROL），支持 `--x/--y/--z` |
| `03_realtime_velocity.py` | 实时速度控制画圆（RealtimeControl） |
| `04_mode_switch.py` | 模式切换与状态打印（默认不解锁，安全） |
| `05_keyboard_teleop.py` | 键盘遥操（需真实终端 TTY） |

## 坐标系约定

- 位置/速度：局部 **ENU**（x 东、y 北、z 上），起飞目标 z 为正即升高。
- 姿态：FLU（前-左-上）四元数。
- MAVROS 已自动完成与 PX4 内部 NED/FRD 的转换，示例无需关心。

## 与 ROS2 版差异

- 客户端用 `rospy`，等待逻辑用 `rospy.sleep` + 轮询（替代 rclpy 的 spin）。
- 其余话题名、消息字段、任务语义与 ROS2 版完全一致。
