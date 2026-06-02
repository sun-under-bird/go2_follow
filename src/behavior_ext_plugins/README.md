# BackUpTwzFree 脱困行为插件

这个包是给 ROS2 Humble + apt 安装版 Navigation2 使用的 overlay 插件，不需要修改 `navigation2` 源码。插件通过 `pluginlib` 挂到 `behavior_server` 的 `backup` 行为上，行为树里仍然使用标准的 `<BackUp .../>` 节点。

## 功能

`BackUpTwzFree` 在触发恢复行为时会：

1. 调用配置的 costmap 服务，例如 `local_costmap/get_costmap`。
2. 从机器人外接圆之外开始逐步扩大搜索半径。
3. 找到代价值低于 `cost_threshold` 的自由栅格。
4. 计算自由栅格质心，并朝这个方向移动 `BackUp` action 指定的距离。
5. 继续复用 Nav2 `DriveOnHeading` 的碰撞前瞻检查。

对只有前方双目相机的机器人，建议优先使用 `local_costmap/get_costmap`，并让 local costmap 把未观测区域保持为 unknown，避免侧后方未观测区域被当成可通行空间。

## 编译方式

把本目录复制到自己的 ROS2 工作空间：

```bash
cp -r nav2_plugins/behavior_ext_plugins ~/nav2_ws/src/
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select behavior_ext_plugins
source install/setup.bash
```

如果你的 Navigation2 是 apt 安装的 Humble 版本，只要已经安装完整 Nav2 依赖即可：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Nav2 参数接入

在自己的 `nav2_params.yaml` 里把 `behavior_server.backup.plugin` 改成：

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]

    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUpTwzFree"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"

    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    cycle_frequency: 10.0

    robot_radius: 0.25
    max_radius: 2.0
    service_name: "local_costmap/get_costmap"
    free_threshold: 5
    cost_threshold: 0.0
    visualization: true
    enable_strafe: true
```

参数说明：

- `service_name`：读取哪张 costmap。只有前方双目时推荐 `local_costmap/get_costmap`。
- `robot_radius`：机器人外接圆半径，搜索自由空间时会跳过机器人自身 footprint。
- `max_radius`：最大搜索半径。
- `free_threshold`：认为找到自由空间所需的最少自由栅格数量。
- `cost_threshold`：costmap 中允许作为自由空间的最大代价值。Nav2 常见自由栅格为 `0`，未知区通常为 `255`。
- `visualization`：是否发布 `back_up_twz_free_markers` 供 RViz 查看。
- `enable_strafe`：全向底盘设为 `true`，插件会发布 `linear.x` 和 `linear.y`；差速底盘设为 `false`，插件只发布 `linear.x`。

## 行为树

行为树可以继续使用 Nav2 标准 BackUp 节点，不需要新增 BT 节点：

```xml
<BackUp backup_dist="0.3" backup_speed="0.2"/>
```

插件替换的是 `behavior_server` 中 `backup` 行为的实现，所以 BT XML 不需要写 `BackUpTwzFree`。

## 双目相机注意事项

双目相机通常输出深度图或 `PointCloud2`。local costmap 建议使用 Nav2 标准 `ObstacleLayer` 或 `VoxelLayer`，不要直接套用本仓库的 `costmap_intensity`，因为那个插件依赖点云 `intensity` 语义。

前方双目视野有限，local costmap 最好配置为 unknown 区域不可被本插件当成自由空间。示例配置见 `examples/humble_stereo_nav2_params.yaml`。
