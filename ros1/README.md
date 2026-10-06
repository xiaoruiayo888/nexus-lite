# nexus-lite · ROS1（Noetic / MAVROS）版

本目录是 `nexus-lite` 的 **ROS 1** 移植版本，与原 ROS 2 工程**并存**（不修改、不覆盖 `src/` 下的 ROS 2 代码）。

- ROS 2 原版链路：Micro XRCE-DDS Agent + `px4_msgs`（直接收发 uORB）+ Gazebo Harmonic/Fortress。
- ROS 1 移植链路：**MAVROS（MAVLink）** + **Gazebo Classic 11**。

> 真正的移植障碍是通信中间件替换，而不是 ROS API 翻译。MAVROS 已自动完成 NED↔ENU 坐标转换，
> 服务调用同步返回成败，并内置 AUTO.RTL/AUTO.LAND，因此 ROS1 版可删除坐标变换、ACK 轮询与自实现返航。

## 目录结构

```
ros1/
├── README.md                       # 本文件
├── catkin_ws/
│   └── src/
│       ├── px4_bridge_msgs/        # 自定义消息（TaskCommand/RealtimeControl/BridgeStatus）
│       ├── px4_bridge/             # 控制桥主包（rospy + MAVROS）
│       │   ├── nodes/              # bridge_node / odom_bridge_node
│       │   ├── src/px4_bridge/     # 节点、状态机、任务、实时控制、MAVROS 适配
│       │   ├── launch/ config/
│       └── px4_sitl_gazebo/        # Gazebo Classic 模型/世界/launch（drone260）
└── examples/                       # rospy 示例 01..05
```

## 一、环境准备（虚拟机 Ubuntu 20.04）

1. 安装 ROS Noetic（官方 wiki 步骤）与 Gazebo Classic 11（随 Noetic 桌面版自带）。
2. 安装 MAVROS 及额外插件：
   ```bash
   sudo apt update
   sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-gazebo-plugins
   # 安装 GeographicLib 数据集（MAVROS 依赖，只需一次）
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   chmod +x install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```
3. 获取并编译 PX4 源码（建议 v1.14.3，对 Gazebo Classic + Noetic 最友好）：
   ```bash
   cd ~
   git clone -b v1.14.3 https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   bash ./Tools/setup/ubuntu.sh
   DONT_RUN=1 make px4_sitl_default gazebo-classic
   source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
   ```
   建议把上面的 `source` 与 `export` 写进 `~/.bashrc`。

## 二、编译本工作空间

把整个 `ros1/` 目录拷到虚拟机后：

```bash
cd ~/nexus-lite/ros1/catkin_ws
catkin_make
source devel/setup.bash
```

> 若 `rosrun` 提示节点不可执行，补一次执行权限：
> `chmod +x src/px4_bridge/nodes/bridge_node src/px4_bridge/nodes/odom_bridge_node`

## 三、运行（三个终端）

**终端 1 —— PX4 SITL + Gazebo Classic（spawn drone260）：**
```bash
cd ~/nexus-lite/ros1/catkin_ws && source devel/setup.bash
roslaunch px4_sitl_gazebo sitl_gazebo.launch
```

**终端 2 —— MAVROS + 控制桥 + 里程计反向桥：**
```bash
cd ~/nexus-lite/ros1/catkin_ws && source devel/setup.bash
roslaunch px4_bridge mavros_bridge.launch
```
（若终端 1 已含 MAVROS 或你单独启动了 MAVROS，可改用 `roslaunch px4_bridge bridge.launch`。）

**终端 3 —— 示例：**
```bash
cd ~/nexus-lite/ros1/examples
python3 01_arm_takeoff_land.py
```

## 四、话题接口（与 ROS2 版一致）

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/px4_bridge/in/task_cmd` | `px4_bridge_msgs/TaskCommand` |
| 订阅 | `/px4_bridge/in/realtime_control` | `px4_bridge_msgs/RealtimeControl` |
| 发布 | `/px4_bridge/out/status` | `px4_bridge_msgs/BridgeStatus` |

MAVROS 侧：状态来自 `mavros/state`、`mavros/local_position/*`、`mavros/imu/data`、`mavros/battery`；
设定值发 `mavros/setpoint_position/local`、`mavros/setpoint_raw/local`、`mavros/setpoint_raw/attitude`；
解锁/切模式用 `mavros/cmd/arming`、`mavros/set_mode` 服务。视觉定位由 odom_bridge 发 `mavros/vision_pose/pose`。

## 五、ROS2 → ROS1 技术映射

| 项 | ROS 2 | ROS 1 |
| --- | --- | --- |
| 系统 | Ubuntu 22.04 / Humble | Ubuntu 20.04 / Noetic（均 Python 3） |
| 构建 | colcon / ament_python | catkin（CMake + catkin_python_setup） |
| PX4 链路 | Micro XRCE-DDS + px4_msgs | MAVROS / MAVLink |
| QoS | BEST_EFFORT + TRANSIENT_LOCAL | 无 QoS（queue_size / TCPROS） |
| Launch | Python | XML（.launch） |
| 仿真 | Gazebo Harmonic/Fortress | Gazebo Classic 11 |
| 视觉里程计 | `VehicleOdometry` → `/fmu/in/vehicle_visual_odometry` | PoseStamped → `mavros/vision_pose/pose` |
| 可视化 | foxglove_bridge | rosbridge_suite（Foxglove 经 rosbridge 连接） |

## 六、已知限制

- **failsafe 粒度更粗**：MAVROS 不提供 ROS2 的 `FailsafeFlags`，ROS1 版以 `mavros/state.system_status`
  处于 CRITICAL/EMERGENCY/FLIGHT_TERMINATION 判定严重故障，reason 标注 system_status。
- **返航简化**：直接切 `AUTO.RTL`（PX4 内置返航降落），不再自行实现"飞 home 上方 + LAND"分段流程。
- **AirSim 未移植**：本次仅做 Gazebo Classic，AirSim 桥以后再处理。
- **验证范围**：当前在 Windows 侧仅完成 Python 语法静态检查（18 个文件 py_compile 通过）与结构核对；
  `catkin_make`、SITL 联机与飞行需你在虚拟机内执行验证。
