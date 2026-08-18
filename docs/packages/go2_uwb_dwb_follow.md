# `go2_uwb_dwb_follow` 使用说明

## 定位与接口

本包提供最低复杂度的 Nav2 标准控制器对照方案。`uwb_point_follow_node` 解析原始 UWB、转换到 `odom` 并计算保持距离站位点，然后发布两点路径；它不再直接持有 `FollowPath` action。

| 方向 | 名称 | 类型/说明 |
| --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | `uwb_aoa_pkg/msg/LibAoaRobotMsg` |
| 发布 | `/uwb_follow_target` | `PointStamped`，`odom` 坐标系 |
| 发布 | `/uwb_dwb/path` | 两点 `Path` |
| 发布 | `/follow/target_valid` | 目标是否有效 |
| 发布 | `/follow/uwb_point_status` | UWB/路径状态 |
| 行为树状态 | `/follow/dwb_recovery_status` | tracking/recovering/failed |
| 最终速度 | `/cmd_vel` | DWB 或恢复行为直接发布 |

## 有效性与站位点

- 默认 `prefer_range_angle=true`，优先使用 `r/a`，否则使用 `x/y`。
- 统一使用 `odom` 和 `base_footprint`；TF 失败不会只改 frame 标签。
- 不检查状态、置信度、距离、单帧跳变或目标速度，也不执行一阶低通。
- 默认保持距离 `1.2 m`，禁止生成机器人后方站位点。
- 收到第一帧后持续复用最后目标；已到站位点时发布空路径，TF 失败的单帧会保留此前目标。

## Nav2 与恢复配置

`nav2_dwb_controller.yaml` 包含 controller、rolling local costmap 和 behavior server：

- DWB：`vx 0~0.45 m/s`、`wz ±1.0 rad/s`、16 个线速度样本、32 个角速度样本、预测 `1.5 s`。
- critics：`RotateToGoal`、`Oscillation`、`BaseObstacle`、`GoalAlign`、`GoalDist`。
- local costmap：4×4 m、0.05 m、Go2 矩形 footprint，使用障碍点云标记、地面点云清除。
- behavior server：标准 `backup` action 绑定 `BackUpTwzFree`。
- 行为树：`RecoveryNode(FollowPath, BackUp)`，默认恢复两次，每次移动 `0.30 m`，速度 `0.12 m/s`。

## 测试重点

1. 无障碍：DWB 应产生平滑的前进/转向采样结果。
2. 路径侧边障碍：轨迹应偏离两点直线但不进入 footprint 碰撞区。
3. 正前方完全阻挡：先停住；进度检查超时后 `/follow/dwb_recovery_status` 变为 `recovering`。
4. 恢复成功：出现 `/backup` action，完成后自动再次进入 `tracking`。
5. 恢复失败：两次后状态为 `failed: recovery retries exhausted`，且不无限循环。

完整实机步骤见 [方案测试矩阵](../FOLLOW_TEST_MATRIX.md)。
