# `go2_dynamic_follow_avoidance` 使用说明

## 包定位

该方案在 D435i 局部点云上建立二维栅格并运行 A*，因此能比 DWB/直连 MPPI 更明确地选择局部绕行通道。MPPI 负责跟踪 A* 路径，行为树负责控制失败后的自由方向恢复。

## 节点

| 节点 | 职责 |
| --- | --- |
| `follow_goal_node` | UWB 原始坐标解析、TF 和保持距离目标 |
| `local_path_planner` | 点云栅格、地面清除、目标投影、局部 A* 和路径有效性 |
| `follow_path_recovery_bt_node` | `FollowPath → BackUpTwzFree → 重试` |
| `simple_follow_controller` | 无感知 P 控制，仅用于 UWB 方向验证 |

已删除的冗余组件：`follow_path_action_client`、`safety_mux`、`safety_helpers`。运行链只使用 `/cmd_vel`，没有速度转发节点。

## 主要话题

| 输入/输出 | 话题 |
| --- | --- |
| UWB | `/libAoa_robot_publisher` |
| 里程计 | `/odom` |
| 机体坐标 | `base_footprint` |
| 障碍/地面点云 | `/local_grid_obstacle`、`/local_grid_ground` |
| 路径 | `/follow_path` |
| 路径有效性 | `/follow/path_valid` |
| 行为树状态 | `/follow/dynamic_recovery_status` |
| 最终速度 | `/cmd_vel` |

路径有效性为 false 时，行为树会立即撤销当前控制/恢复 action。有效路径终点变化超过阈值时才更新 `FollowPath`，避免频繁抢占。

## 模式

- 标准模式：A* + MPPI + behavior server，可局部避障并恢复。
- simple 模式：直接按目标距离/角度发布 `/cmd_vel`，不使用点云、costmap 或恢复。必须同时设置 `start_nav2_controller:=false`。

## 当前取舍

本阶段目标是先验证基本跟随和避障，因此没有安全门控、输入超时 watchdog 或速度平滑。收到第一帧 UWB/点云后会持续复用最后数据；空路径只表示当前没有目标或已经进入跟随距离。

完整测试步骤见 [方案测试矩阵](../FOLLOW_TEST_MATRIX.md)。
