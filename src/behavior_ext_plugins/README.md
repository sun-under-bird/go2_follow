# FollowPath 恢复行为树与空闲方向脱困

该包包含两个配套组件：

1. `nav2_behaviors/BackUpTwzFree`：`nav2_core::Behavior` 插件，从 local costmap 中寻找自由栅格方向并执行移动。
2. `follow_path_recovery_bt_node`：订阅路径并执行 `RecoveryNode(FollowPath, BackUp)` 的行为树。控制失败后调用标准 `/backup` action，恢复成功后重试最新路径。

行为树文件：`behavior_trees/follow_path_with_free_space_recovery.xml`。

## 行为树结构

```xml
<RecoveryNode number_of_retries="{recovery_retries}">
  <FollowPath path="{path}" controller_id="{controller_id}"/>
  <BackUp backup_dist="{recovery_distance_m}"
          backup_speed="{recovery_speed_mps}"/>
</RecoveryNode>
```

`BackUp` 是 Nav2 标准 BT 节点；只有 behavior server 中的实现被替换为空闲方向插件：

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: [backup]
    backup:
      plugin: nav2_behaviors/BackUpTwzFree
    global_frame: odom
    robot_base_frame: base_footprint
    service_name: local_costmap/get_costmap
    enable_strafe: true
```

插件会扫描机器人半径外、`max_radius` 内且代价不高于阈值的栅格，计算满足最小自由格数量的近邻区域质心，并按质心方向发布速度。全向模式可发布 `linear.y`；碰撞前瞻失败、超时或达到移动距离时立即停止。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select behavior_ext_plugins
source install/setup.bash
```

该执行器已由 DWB、直连 MPPI、Dynamic A*+MPPI 和 TEB launch 自动启动，一般无需单独运行。
