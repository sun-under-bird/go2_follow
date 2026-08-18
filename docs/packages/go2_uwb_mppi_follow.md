# `go2_uwb_mppi_follow` 使用说明

## 包定位

该方案专门验证“只根据机器人和 UWB 站位点生成直线路径，再由 MPPI 跟踪并局部避障”。它没有 planner server 或 global costmap，因此不会主动生成绕过大障碍的全局路径。

## 节点职责

### `one1000_target_point_node`

把 `LibAoaRobotMsg` 转为 `/uwb/target_point`：仅做字段解析、有限值检查和必要的 TF，支持 `r/a` 与 `x/y` 回退，不使用状态、置信度或距离门限。当前输入、输出 frame 均为 `base_footprint`。

### `uwb_follow_path_node`

直接使用最新原始 UWB 点执行站位点计算和直线路径插值，发布：

| 话题 | 说明 |
| --- | --- |
| `/uwb_follow/target_raw` | 未经低通、跳变、速度、置信度和超时门控的人员点 |
| `/uwb_follow/follow_goal` | 扣除 `1.5 m` 后的站位点 |
| `/uwb_follow/path` | `odom` 中的直线路径 |
| `/follow/target_valid` | 当前目标有效性 |
| `/follow/uwb_path_status` | UWB/路径状态 |
| `/cmd_vel` | 只在启用近距离原地转向时发布；普通停车由空路径撤销控制 action |

路径以固定频率保活；行为树执行器比较终点位置/朝向后决定是否更新 `FollowPath`，因此不会因起点或时间戳变化持续抢占控制器。

### `target_obstacle_filter_node`

从 `/local_grid_obstacle` 中删除真实人员点附近有限圆柱，输出 `/local_grid_obstacle_filtered`。收到第一帧目标后持续使用该目标，直到新目标覆盖。

### `follow_path_recovery_bt_node`

订阅 `/uwb_follow/path`，执行 `RecoveryNode(FollowPath, BackUp)`。`BackUp` 的底层插件是 `BackUpTwzFree`，会从 `local_costmap/get_costmap` 选择自由方向。状态发布在 `/follow/mppi_recovery_status`。

## MPPI 参数摘要

- DiffDrive：`vx 0~0.45 m/s`、`wz ±1.0 rad/s`。
- `model_dt=0.05`、40 步、1000 批样本。
- 主要 critics：Constraint、Obstacles、Goal、PathFollow、PathAngle、PreferForward。
- rolling local costmap：4×4 m、0.05 m、`odom`/`base_footprint`。
- STVL 使用过滤后的障碍云标记、地面云清除，膨胀半径 `0.4 m`。
- controller 和恢复 behavior 都直接发布 `/cmd_vel`，无 velocity smoother。

## 恢复触发语义

代价地图预测碰撞时 MPPI 不会发布穿过障碍的速度；若短期内还能找到有效采样则会局部绕行。只有控制器最终返回失败（例如无有效控制或进度检查超时）后，行为树才进入空闲方向恢复。恢复完成后自动重试最新路径；两次均失败则停止等待新目标。

完整启动和实测步骤见包内 README 与 [方案测试矩阵](../FOLLOW_TEST_MATRIX.md)。
