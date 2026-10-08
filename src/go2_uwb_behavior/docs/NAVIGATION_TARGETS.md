# 只生成导航目标，运动交给上层 Nav2

`ros2 launch go2_uwb_behavior navigation_targets.launch.py` 启用 `target_only=true`。
此模式不会创建 `/cmd_vel` 或名义速度发布者，旧 FollowUwb、OrbitUwbOnce、RandomRoam
运动 Goal 会被拒绝。采样时短暂开启双目/障碍计算，响应后关闭；Nav2 自己提供导航感知。
原 `behavior_follow_roam.launch.py` 默认仍是旧运动模式，目标模式节点名为 `/uwb_navigation_target_node`，运动模式仍为
`/uwb_behavior_controller_node`。两种模式共享服务/感知话题，不能同时启动。
切换模式需停止旧启动链、重新启动；持续 UWB 跟随继续使用旧运动模式。

接口：`/go2/generate_navigation_goal`，类型 `go2_uwb_behavior/srv/GenerateNavigationGoal`。
请求包含唯一 `request_id`、`random_seed`、`min_radius`、`max_radius`。
两个半径均为零表示用配置默认值；否则必须落在配置许可范围内。
返回原 `request_id`、`success`、`code`、`message`、`navigation_goal`、`center_pose`。
两个位姿默认均在 `map` 坐标系。缺少新鲜 UWB/里程计/障碍输入或 map←odom TF 时明确失败。
圆心优先冻结在请求接收时；当时输入不齐则冻结在本次输入就绪时。
采样保留已有圆环范围与障碍净空检查，不使用地图命名点位 K。

`SUCCESS=0` 仅表示目标生成成功，不表示机器人已到达。
`/go2/navigation_target` 的 PoseStamped 仅供显示；上层不能订阅它就自动运动。
正式导航必须使用本次服务响应，通过 `request_id` 关联，并检查成功业务码、坐标系及时间戳。

同一时刻只处理一个目标请求，第二个请求返回 BUSY；等待传感器/TF 是有限的。
`/go2/set_behavior` 的 IDLE/STOP 会终止待处理请求；STOP 保持锁存，下一次生成前需明确解锁。
运动模式调用目标服务返回 UNSUPPORTED_MODE，禁止偷偷混用两条速度链。

上层时序：Tree 的一个 Behavior Goal → Action 请求目标 → Action 发送一个
Nav2 NavigateToPose → 等待真实 Result → 确认底盘停稳 → 后续姿态动作。
Action 负责目标时效、控制权、导航超时、取消、结果与故障恢复锁。
Nav2 使用自己的代价地图和路径；目标采样净空并不保证最终导航成功。
旧局部规划器的逐速度预测围栏不适用于此模式，上层必须自行约束 Nav2 路径/运动。
MarsDogAction 会按 Nav2 位姿反馈检查相对冻结圆心的 6 米范围；反馈停车存在延迟，
不等同于旧链的逐速度预测围栏，实机需验证取消制动距离。
