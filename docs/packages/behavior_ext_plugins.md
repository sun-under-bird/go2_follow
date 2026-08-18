# `behavior_ext_plugins` 使用说明

## 组件一：`BackUpTwzFree`

这是 behavior server 插件，不是 BT XML 节点。它仍通过标准 `/backup` action 被 `<BackUp/>` 调用。

处理步骤：

1. 从 `local_costmap/get_costmap` 获取当前地图。
2. 取得 `odom -> base_footprint` 位姿。
3. 单次扫描机器人半径外、最大搜索半径内的自由栅格并按距离排序。
4. 选出满足 `free_threshold` 的最小邻域，计算自由点质心。
5. 全向模式朝质心发布 x/y；非全向模式选择更接近质心的前进或后退方向。
6. 复用 Nav2 `DriveOnHeading` 碰撞前瞻，达到距离、超时或将碰撞时停止。

关键参数：`robot_radius`、`max_radius`、`service_name`、`free_threshold`、`cost_threshold`、`visualization`、`enable_strafe`。

## 组件二：`follow_path_recovery_bt_node`

该节点让直接发布 `Path` 的跟随方案获得真正的恢复链，而不必改成 `NavigateToPose` 或增加全局 planner。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `path_topic` | `/follow_path` | 行为树输入路径 |
| `path_valid_topic` | 空 | 可选 `Bool`，false 时撤销 action |
| `status_topic` | `/follow/recovery_status` | tracking/recovering/final 状态 |
| `recovery_retries` | 2 | 最大恢复次数 |
| `recovery_distance_m` | 0.30 | 单次脱困距离 |
| `recovery_speed_mps` | 0.12 | 脱困速度幅值 |
| `path_timeout_sec` | 0.0 | 路径保活超时；0 表示关闭，持续复用最后路径 |
| `goal_update_distance_m` | 0.08 | 更新 action 的终点位移阈值 |
| `goal_update_angle_rad` | 0.10 | 更新 action 的终点朝向阈值 |

节点只比较路径终点，不因时间戳或机器人起点变化高频抢占。空路径会立即 halt 行为树。恢复次数耗尽后，它记住失败终点并等待目标明显变化，避免无限恢复循环。

## 观测

```bash
ros2 action list | grep -E 'follow_path|backup'
ros2 topic echo /follow/dwb_recovery_status
ros2 topic echo /back_up_twz_free_markers
ros2 topic info /cmd_vel --verbose
```

已知限制：自由区域当前使用近邻质心，不评估最大连通区域或完整通道宽度；前向 D435i 未观测的侧后方必须在 costmap 中保留为 unknown，不能当作自由区域。
