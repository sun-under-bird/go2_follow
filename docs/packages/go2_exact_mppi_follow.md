# `go2_exact_mppi_follow` 使用说明

## 包定位

该 Python 包把双目 PointCloud2 直接筛选成二维障碍点，送入固定版本 EXACT-MPPI 的 JAX `MPPIController`，不使用 Nav2 costmap。它适合 Jetson Orin 上的控制算法实验，默认要求 JAX GPU 后端。

## 第三方版本

仓库 `third_party/EXACT-mppi.PINNED` 固定：

- 仓库：`https://github.com/caseypen/EXACT-mppi`
- commit：`54dcd24ae7284f6779adb8e72cc702c3a6d65095`
- 安装子目录：`EXACT_MPPI_core`

安装示例：

```bash
cd ~/go2_follow
git clone https://github.com/caseypen/EXACT-mppi.git third_party/EXACT-mppi
cd third_party/EXACT-mppi
git checkout 54dcd24ae7284f6779adb8e72cc702c3a6d65095
python3 -m pip install -U "jax[cuda12]"
python3 -m pip install -e ./EXACT_MPPI_core
```

Jetson 的 JAX wheel 必须与 JetPack/CUDA 匹配，不能仅按桌面 CUDA 命令盲装。启动前检查：

```bash
python3 -c "import jax; print(jax.default_backend(), jax.devices())"
python3 -c "from exact_mppi.mppi_jax.controller import MPPIController; print(MPPIController)"
```

## 数据流

```text
/libAoa_robot_publisher
  -> go2_uwb_goal_bridge
  -> /exact_mppi/goal_local

/local_grid_obstacle -> TF/ROI/footprint/voxel/选点 ┐
/odom ------------------------------------------┼-> go2_exact_mppi_node
/exact_mppi/goal_local -------------------------┘
  -> MPPIController.computeVelocityCommands
  -> /cmd_vel
```

## `go2_uwb_goal_bridge`

### 接口

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | `LibAoaRobotMsg` |
| 发布 | `/exact_mppi/goal_local` | `PoseStamped`，base frame |
| 发布 | `/exact_mppi/goal_status` | `String` |

### 行为

- 不检查 `state`、置信度或目标距离门限，仅拒绝非有限坐标。
- 优先 x/y，回退 r/a；支持角度单位、轴翻转和锚点偏移。
- 使用消息 header 或 fallback frame，通过 TF 转到 `base_footprint`。
- 将人员目标沿方位缩短 `follow_distance=0.9 m`。
- 人员过近时允许最多 `0.4 m` 反向局部 goal，过远时前向 goal 最多 `2.0 m`。
- 该节点没有目标低通、跳变、置信度或超时门控，持续复用最后一个 goal。

## `go2_exact_mppi_node`

### 输入输出

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/local_grid_obstacle` | `PointCloud2` |
| 订阅 | `/odom` | `Odometry` |
| 订阅 | `/exact_mppi/goal_local` | `PoseStamped` |
| 发布 | `/cmd_vel` | 最终 `Twist` |
| 发布 | `/exact_mppi/filtered_points` | 筛选后的二维障碍 PointCloud2 |
| 发布 | `/exact_mppi/status` | `String` |
| 发布 | `/exact_mppi/cost_breakdown` | `Float32MultiArray`，label 中携带 critic key |
| 发布 | `/exact_mppi/plan` | 最优轨迹 `Path` |

### 点云处理

1. 最多读取 `max_raw_points=60000`。
2. 按点云消息时间查 TF，失败时回退 latest。
3. 默认 ROI：`x -0.4~3.0 m`、`|y|<=1.8 m`、`z 0.05~1.0 m`。
4. 删除 footprint AABB 内的点。
5. 按 `voxel_size=0.08 m` 对 xy 降采样。
6. 对机器人近点和目标走廊点提高优先级，最多保留 MPPI `max_obs_num=300` 个点。

### 控制

- 10 Hz MultiThreadedExecutor，控制 callback 与输入 callback 分组。
- goal/cloud/odom 各收到第一帧后持续复用最后数据，不做输入超时停车。
- 以机器人局部原点作为当前姿态，根据当前速度、直线参考 plan、goal 和障碍调用 EXACT-MPPI。
- 最近障碍 `<0.18 m` 时在控制器外急停。
- omni action 映射为 `linear.x`、`linear.y`、`angular.z`。
- 可选发布最优轨迹、critic 均值和周期耗时。

## MPPI 配置

`go2_orin_omni.yaml` 默认：

- omni，`dt=0.1 s`、20 步、256 样本、1 次迭代。
- `vx -0.3~0.6`、`vy ±0.4`、`wz ±0.8`。
- footprint：前 0.45、后 0.38、左右 0.23 m。
- 碰撞余量 0.20 m。
- critics：Constraint、Goal、GoalAngle、Obstacles、PathAlign、PathFollow、PathAngle、VelocityDeadband、Twirling；PreferForward 关闭。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install \
  --packages-select uwb_aoa_pkg go2_exact_mppi_follow
source install/setup.bash

ros2 launch go2_exact_mppi_follow go2_exact_mppi_follow.launch.py \
  pointcloud_topic:=/local_grid_obstacle \
  odom_topic:=/odom \
  one1000_topic:=/libAoa_robot_publisher
```

CPU 仅限离线调试时，可在参数文件将 `require_jax_gpu` 改为 false；实机不应在未测定周期抖动的 CPU 模式运行。

## 检查

```bash
ros2 topic echo /exact_mppi/goal_status
ros2 topic echo /exact_mppi/status
ros2 topic hz /exact_mppi/filtered_points
ros2 topic echo /exact_mppi/cost_breakdown --once
ros2 topic echo /exact_mppi/plan --once
ros2 topic echo /cmd_vel
```

日志中的 `mppi_tick=...ms` 应显著小于 100 ms，且最坏耗时也要满足 10 Hz 周期。

## 已知边界

- 包没有自动安装 JAX/EXACT-MPPI，也没有 JetPack 版本矩阵。
- package.xml 未声明 NumPy、PyYAML、JAX 和 exact_mppi 的完整系统依赖。
- 最终命令直接发布 `/cmd_vel`；没有独立加速度平滑、速度 mux 或 footprint 安全门控。
- UWB goal 没有跳变、置信度或超时过滤，异常 goal 会一直保留到新目标覆盖或节点停止。
- 点云筛选使用 Python list/NumPy 转换，需在 Orin 上剖析内存复制和 GIL 影响。
- `map_frame` 参数被加载但当前控制计算基于完全局部坐标，未实际参与规划。
- cost breakdown 把 key 编码在 MultiArray label 中，不适合稳定机器接口。
