# `go2_uwb_teb_follow` 使用说明

## 定位

这是项目中唯一同时使用 planner server 和 controller server 的 UWB 路线。Smac Hybrid 在 rolling global costmap 中规划，TEB 负责速度优化，`BackUpTwzFree` 负责控制失败后的自由方向恢复。

## 关键接口

| 名称 | 用途 |
| --- | --- |
| `/libAoa_robot_publisher` | UWB 输入 |
| `/compute_path_to_pose` | Smac 规划 action |
| `/uwb_teb/target` | 人员点，供人体点云过滤 |
| `/uwb_teb/path` | 规划结果及行为树输入 |
| `/follow/uwb_teb_status` | UWB/规划状态 |
| `/follow/teb_recovery_status` | 行为树状态 |
| `/cmd_vel` | TEB 或恢复行为最终速度 |

## 跟随与恢复

- 不检查 UWB `state` 或置信度，优先 `r/a`，回退 `x/y`，收到后持续复用最后目标。
- 站位点等于人员方向上扣除 `follow_distance_m=1.0` 的位置，不把人员中心当规划终点。
- 目标移动超过 `0.10 m` 或方位变化超过约 `5°` 才重新请求路径。
- 规划返回过慢、路径过短或目标已经移动时丢弃旧结果。
- 路径发布后由共享行为树执行 `FollowPath`；失败后运行自由方向 `BackUp` 并重试两次。

## 配置摘要

- global/local costmap 均使用 `odom` 和 `base_footprint`。
- 障碍云为 `/local_grid_obstacle_filtered`，地面云用于 clearing。
- planner 为 `SmacPlannerHybrid`，controller 为配置的 TEB 插件。
- behavior server 的 `backup` 绑定 `nav2_behaviors/BackUpTwzFree`。
- 不启动 velocity smoother，所有速度直接发布 `/cmd_vel`。

完整物理测试步骤见 [方案测试矩阵](../FOLLOW_TEST_MATRIX.md)。
