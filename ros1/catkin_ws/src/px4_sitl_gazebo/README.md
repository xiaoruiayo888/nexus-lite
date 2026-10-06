# px4_sitl_gazebo（Gazebo Classic / PX4 SITL）

为 ROS Noetic 提供 **Gazebo Classic 11** 下的 `drone260` 模型、世界与启动文件，替代 ROS2 版的 Gazebo Harmonic/Fortress 资源。

## 资源

```
px4_sitl_gazebo/
├── launch/sitl_gazebo.launch      # PX4 SITL + Gazebo + spawn drone260
├── worlds/custom.world            # 地面 + 光照（VM 默认关阴影）
└── models/drone260/
    ├── model.sdf                  # classic 插件版（见下）
    ├── model.config
    └── meshes/                     # drone/motor/mid360 网格（自包含）
```

`model.sdf` 使用的插件：

| 插件 | 作用 |
| --- | --- |
| `libgazebo_mavlink_interface.so` | PX4↔Gazebo MAVLink + lockstep，开放 14540 给 MAVROS |
| `libgazebo_motor_model.so` ×4 | 电机/旋翼模型 |
| `libgazebo_imu_plugin.so` / `libgazebo_magnetometer_plugin.so` / `libgazebo_barometer_plugin.so` / `libgazebo_gps_plugin.so` | 飞控传感器（数据走 MAVLink） |
| `libgazebo_ros_p3d.so` | 直接发布 `/drone260/odom`（nav_msgs/Odometry），供 odom_bridge |
| `libgazebo_ros_block_laser.so` | MID360 近似点云 `/drone260/lidar_points`（PointCloud2） |

## 两种使用路径

### 路径 A：复用 iris 机架参数（默认，零改 PX4 源码，推荐）

`drone260` 是标准 X 型四旋翼，混控/传感器与 PX4 自带 `iris` 一致。launch 默认令 PX4 加载 `gazebo-classic_iris` 的飞控参数，而 Gazebo 中 spawn 我们自己的 `drone260` 模型与传感器，即可正常飞行。

```bash
roslaunch px4_sitl_gazebo sitl_gazebo.launch
```

### 路径 B：在 PX4 中登记独立的 drone260 机架

若希望 PX4 内机架名即为 `drone260`（独立参数调参）：

1. 复制 classic 的 iris 机架文件作为模板（在 PX4-Autopilot 源码内）：
   ```bash
   cd ROMFS/px4fmu_common/init.d-posix/airframes
   cp <iris 对应文件，如 4010_gazebo-classic_iris> 4099_gazebo-classic_drone260
   ```
   新文件内设置四旋翼 X 混控与 MAVLink（与 iris 相同即可），按需调参。
2. 在同目录 `CMakeLists.txt` 的机架列表里注册 `4099_gazebo-classic_drone260`。
3. 重新编译：
   ```bash
   cd ~/PX4-Autopilot
   DONT_RUN=1 make px4_sitl_default gazebo-classic
   ```
4. 启动时覆盖机架参数：
   ```bash
   roslaunch px4_sitl_gazebo sitl_gazebo.launch sim_model:=gazebo-classic_drone260
   ```

> 模型 SDF 由本包提供并 spawn，因此路径 B 无需把 SDF 放进 PX4 源码，只需登记 airframe 参数。

## PX4 版本建议

Gazebo Classic 在 **PX4 v1.14.x** 上内置且与 Noetic 配合最稳定；v1.15 起 classic 改为外部仓库 `PX4/PX4-SITL_gazebo-classic`。建议虚拟机使用 v1.14.3。
