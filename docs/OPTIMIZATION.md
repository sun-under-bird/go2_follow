# 项目优化记录与后续建议

本文记录本轮已经落地的结构、安全和性能优化，以及仍值得继续推进的事项。优先级定义：P0 影响实机安全或数据正确性，P1 影响可靠性与维护成本，P2 是性能和工程质量提升。

## 已完成的优化

### 1. 统一最终速度出口

所有可运行的跟随方案都直接发布 `geometry_msgs/msg/Twist` 到 `/cmd_vel`。已删除旧中间速度话题、单纯速度转发、Dynamic `safety_mux` 和 MPPI/TEB `velocity_smoother` 运行链。

注意：统一话题并不等于支持多个控制器并行。部署时仍必须一次只启动一条路线，并用下面的命令确认唯一发布者：

```bash
ros2 topic info /cmd_vel --verbose
```

### 2. 收敛当前实验阶段的控制链

- 所有路线不再使用 UWB 状态、置信度、连续样本、距离、跳变、速度或超时门控，直接消费原始目标坐标。
- APF/VFH、轻量 C++ 和 EXACT 路线不再检查目标、点云、scan 或 odom 的输入年龄，收到第一帧后复用最后数据。
- 删除控制命令低通、目标低通、加速度变化率限制和 Nav2 velocity smoother，最终速度直接发布 `/cmd_vel`。
- 保留 TF、有限值、跟随距离以及障碍碰撞/急停逻辑；这些是坐标正确性和避障算法本体，不是独立安全门控。

### 3. 修正 UWB 坐标与时间语义

- 外接 UWB 已因实机无数据问题回退到厂家 ARM64 算法库版本；C5 原始测量先经过厂家 `algo_uwb_aoa_merge`，再发布坐标。
- MPPI、TEB、EXACT、Dynamic、APF/VFH、DWB 和 jie 的默认回退坐标统一为 `base_footprint`。
- `go2_uwb` 和 MPPI 目标适配器禁止“只替换 frame_id、不变换坐标”；TF 失败时拒绝目标。
- 多数链路默认使用消息时间戳查询 TF，避免旧传感器数据与最新姿态错误拼接。

### 4. 相机链路统一为 D435i 已矫正输入

- 删除整个旧 `go2_stereo_camera` 拼接相机包及 V4L2、拆图、`image_proc`、`stereo_image_proc` 依赖。
- 所有内置感知入口统一订阅 `/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw` 及对应 CameraInfo。
- RTAB-Map 直接把上述已矫正双目输入转换为 `/local_grid_obstacle` 和 `/local_grid_ground`。
- MPPI 删除重复的 `stereo_cloud_filter_node`；所有控制器统一消费 RTAB-Map 局部点云。

### 5. 修复 MPPI/TEB 的近目标和人体障碍问题

- MPPI 近距离默认不再绕过 costmap 直接旋转。
- TEB 不再把人员真实位置当规划终点，而是规划到保持 `follow_distance_m` 的站位点。
- TEB 复用目标障碍过滤器，只在受限半径内清除目标人体点云，局部和全局 costmap 都消费过滤结果。

### 6. 删除冗余与死代码

- 删除从未构建的 `uwb_path_tracker_node.cpp/.hpp`。
- 删除源码树中的 rosidl 生成头、旧 UWB 示例脚本、布局 XML 和过时说明。
- EXACT-MPPI 删除没有消费者的 `/exact_mppi/cmd_vel_raw` 发布器。
- UWB 消息接口可在 x86_64 构建，但厂家驱动和静态库仅在 ARM64 Go2 容器构建；开发机不再承担驱动可执行文件验证。

### 7. 降低恢复行为扫描开销

`behavior_ext_plugins` 原先对每个候选半径重复扫描整张 costmap；现在单次扫描并按距离排序，再逐半径选取自由点，显著减少大地图上的重复遍历。

### 8. 接入真实 FollowPath 恢复行为树

- 新增共享 `follow_path_recovery_bt_node` 和 XML：`RecoveryNode(FollowPath, BackUp)`。
- DWB、直连 MPPI、Dynamic A*+MPPI、TEB 都由行为树持有 `FollowPath`，失败后调用 `BackUpTwzFree` 并重试两次。
- 空路径或路径有效性为 false 时立即撤销 action；恢复耗尽后等待目标变化，避免无限脱困循环。
- 修复了标准 BackUp action 传入负速度时自由方向被反转的问题。

## 仍需优先处理

### P0-1：建立运行时唯一发布者保护

当前架构按“一个 launch 对应一个方案”工作，没有额外 relay。ROS 2 本身不会阻止第二个控制器发布 `/cmd_vel`。建议部署层加入启动互斥检查，发现 `/cmd_vel` 已有非底盘发布者时拒绝激活；如果未来需要热切换，再引入带优先级、心跳、急停和零速过渡的正式仲裁器。

验收：任何标准启动方式下 `/cmd_vel` 只有一个控制发布者，误启动第二条路线会立即报错且保持停车。

### P0-2：量化原始 UWB 抖动

当前代码按要求完全忽略 `state` 和置信度。建议采集静止、遮挡、非视距、快速移动和标签掉线 rosbag，比较原始目标与 `/cmd_vel` 的频谱和峰值，为后续是否恢复最小限度的异常值处理提供数据，而不是先凭经验加门限。

验收：所有路线消费同一段回放时目标坐标一致，并能量化原始噪声对速度抖动的影响。

### P0-3：记录移除断流停车后的风险

需要对 UWB、点云、odom、TF 和控制 action 分别断流，确认哪些路线会保持最后命令、哪些会因规划失败变零，并把结果作为实机测试前置条件。当前不能再假定断流后 `/cmd_vel` 自动归零。

### P0-4：验证机器人实际底盘接口

本轮按用户要求统一到 `/cmd_vel`，但 Go2 驱动的实际订阅类型、QoS、坐标方向、限速和 watchdog 必须在目标版本上核对。首次接入应架空或限速到 `0.1 m/s`，验证正负方向和急停。

## P1：架构与可靠性

### 1. 提取共享 UWB 目标适配组件

各方案仍各自实现 `x/y` 与 `r/a` 回退和 TF。建议提取唯一 `uwb_target_adapter`，只发布标准原始 `PointStamped` 与诊断；下游控制器不再直接依赖厂商消息。进一步可把消息定义拆成 `uwb_aoa_msgs`，驱动单独放入 ARM64 专用包。

### 2. 收敛为三个正式 profile

仓库适合保留算法对照，但部署入口不宜有十种。建议正式支持：

- 低算力：VFH。
- 通用推荐：Dynamic A* + Nav2 MPPI + FollowPath 恢复行为树。
- 研究平台：EXACT-MPPI。

DWB、纯 APF、早期 `go2_uwb`、`jie_deamon` 和 TEB 可保留为实验/回归 profile，并在 launch 名称或文档中明确等级。

### 3. 统一点云前处理和 QoS

建议用一个组件完成 TF、ROI、体素降采样、地面分割和目标人体清除，下游消费统一点云或栅格。所有传感器输入使用 SensorDataQoS，目标/状态/action 使用 reliable，并增加发布订阅 QoS 发现测试。

### 4. 参数关系校验和生命周期自检

各控制器需要统一检查：跟随距离、急停/减速/释放距离的大小关系，最大速度/加速度，ROI，frame/topic 非空，action server、costmap 插件和输入频率。建议感知与控制节点改为 lifecycle，只有 TF、标定和输入健康时才进入 active。

### 5. 恢复行为彻底异步化

`BackUpTwzFree::onRun` 仍同步等待 costmap 服务 future，在不合适的 executor/回调组配置下可能超时。建议改为异步状态机，并把“自由区域质心”升级为最大连通自由区与方向变化联合评分。

### 6. 清理包目录和命名

- 将 `src/Follow` 改为与包名一致的 `src/go2_dynamic_follow_avoidance`。
- 明确 `jie_deamon` 拼写是兼容保留，或在一次有版本边界的迁移中改名。
- 统一包版本、维护者、许可证和 README 模板。

目录重命名会影响现有脚本和外部工作区，建议作为独立迁移提交处理，本轮未贸然修改。

## P2：性能、测试与工程质量

- Python A* 点云链路改为 NumPy/组件化 TF，避免逐点 Python 处理。
- `jie_deamon` 的局部障碍记忆由 vector 近邻合并改为哈希栅格。
- 用行为树执行器统一按路径终点位移/角度阈值更新 FollowPath，避免固定周期抢占 action。
- 建立 UWB 串口二进制回放、TF、相机错误标定、奇数宽度和所有零速策略测试。
- 用 rosbag 建立无遮挡、横穿、窄门、U 形障碍、目标丢失和定位跳变场景回归。
- CI 固定执行 `colcon build`、`colcon test`、launch 加载、YAML 校验和 `/cmd_vel` 唯一发布者检查。
- 将自由文本状态逐步替换为 `diagnostic_msgs/DiagnosticArray`，包含输入年龄、TF 失败次数、点数、规划耗时和停车原因。

## 推荐实施顺序

1. 实机前：完成底盘接口核验、UWB 状态标定和全链路断流零速测试。
2. 下一阶段：抽取共享 UWB 适配器、增加启动互斥保护和 lifecycle 自检。
3. 随后：统一点云前处理，收敛三种正式 profile，并补 rosbag 回归与 CI。
4. 持续优化：性能剖析、参数自动标定、诊断可视化和版本化发布。
