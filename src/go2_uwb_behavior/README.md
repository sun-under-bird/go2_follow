# RK/Lite3 UWB 按需行为控制

本包为上层语音或任务节点提供两个互斥的 ROS 2 Action：

- `/go2/follow_uwb`：持续 UWB 跟随避障，直到上层取消或超时。
- `/go2/random_roam`：冻结调用时的 UWB 位置为圆心，在指定半径圆环内随机选择一个目标，走到并停稳后自动结束。

节点默认 `IDLE`。只有 Action 执行期间才开启双目视差、点云投影和后续避障计算；任务结束、取消或失败后立即关闭计算门控并保持零速度。完整字段与上层时序见
[上层调用接口文档](docs/UPSTREAM_API.md)；**部署、启动命令与验收清单见
[上层对接与部署](docs/上层对接与部署.md)**。

## 控制链路

```text
uwb_behavior_controller_node
  -> /go2_uwb_local_follow/nominal_cmd
  -> local_velocity_planner_node
  -> /go2_uwb_behavior/planner_cmd_vel
  -> uwb_behavior_controller_node 安全门控
  -> /cmd_vel
```

不要同时启动原 `local_follow.launch.py`，否则会出现速度发布者竞争。

## 启动

先启动 UWB、相机和 `/leg_odom2`，再启动统一链路：

```bash
ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py \
  enable_motion:=true cmd_vel_topic:=/cmd_vel
```

首次联调使用 `enable_motion:=false`，接口、点云、规划和诊断仍运行，但最终底盘速度始终为零。

## 上层调用

“跟着我”：

```bash
ros2 action send_goal /go2/follow_uwb \
  go2_uwb_behavior/action/FollowUwb \
  "{timeout_sec: 0.0}" --feedback
```

`timeout_sec: 0.0` 表示持续到上层 cancel；语音“停止跟随”应取消该 Goal。

“原地玩耍”，以调用时 UWB 原点为圆心，在 0.5～2.0 米内走到一个随机点：

```bash
ros2 action send_goal /go2/random_roam \
  go2_uwb_behavior/action/RandomRoam \
  "{random_seed: 0, timeout_sec: 30.0, min_radius: 0.5, max_radius: 2.0}" \
  --feedback
```

`min_radius` 和 `max_radius` 同时为 `0` 时使用配置默认值 0.5～2.0 米。服务端始终限制目标不小于安全下限、不大于 2.0 米，并保留 6 米硬围栏。

锁存停车与解除：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 2}"
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 0}"
```

`SetBehavior` 只接受 `IDLE` 和 `STOP`；`FOLLOW`、`ROAM` 必须通过 Action 发起，便于上层取消、查看反馈和获取可靠终态。

## 按需计算机制

行为节点和 UWB 适配器轻量常驻。Action 接受后，行为节点发布瞬态本地门控
`/go2_uwb_behavior/compute_enable=true`，投影节点开始订阅 disparity；任务结束后断开订阅。
RK 上的 `stereo_image_proc/disparity_node` 在**没有输出订阅者时会跳过双目匹配**（实测：空闲约 2% 单核，出图时约 1.4 核 / 640×480），因此空闲状态不会进行双目匹配或点云投影。滚动地图和规划器无新点云时只保留轻量超时检查。

> 注意：上游的图像订阅在 ROS 图上仍然可见（并未真正退订），所以**不要用「图上还有没有图像订阅」判断双目是否在跑**，要看 CPU 或诊断。另外 RViz、`rosbag record` 等第三方订上 `/stereo/disparity` 会让双目在空闲时重新开始计算。

安全上不能只在开始时计算一帧点云：机器人运动期间环境会变化，所以 Action 活动期间持续感知，停车后才关闭。

## 验证

```bash
colcon build --symlink-install --packages-up-to go2_uwb_behavior
colcon test --packages-select go2_uwb_local_follow go2_uwb_behavior
colcon test-result --verbose
```

实机首次测试建议保持 `enable_motion:=false`，确认以下条件后再放开运动：

1. 空闲时 `/stereo/disparity` 没有投影节点订阅，Action 活动时恢复订阅。
2. `/cmd_vel` 与 `/go2_uwb_local_follow/nominal_cmd` 各只有一个发布者。
3. 取消、STOP、输入断流和障碍阻断均能输出零速并结束任务。
4. 在封闭场地先使用 0.5～2.0 米圆环和 `0.35 m/s` 漫游速度测试。
