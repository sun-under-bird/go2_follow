# Go2 EXACT-MPPI UWB 跟随

该包把双目点云直接筛成局部二维障碍点，并送入 JAX EXACT-MPPI，不使用 Nav2 costmap。它面向 Jetson Orin 等 GPU 研究平台，最终速度只发布到 `/cmd_vel`。

## 输入输出

- UWB：`/libAoa_robot_publisher`。
- D435i RTAB-Map 障碍点云：`/local_grid_obstacle`。
- 里程计：`/odom`。
- 局部站位目标：`/exact_mppi/goal_local`。
- 筛选障碍点：`/exact_mppi/filtered_points`。
- 最优轨迹：`/exact_mppi/plan`。
- 最终速度：`/cmd_vel`。

## 第三方依赖

仓库 `third_party/EXACT-mppi.PINNED` 固定上游 commit。安装示例：

```bash
git clone https://github.com/caseypen/EXACT-mppi.git third_party/EXACT-mppi
cd third_party/EXACT-mppi
git checkout 54dcd24ae7284f6779adb8e72cc702c3a6d65095
python3 -m pip install -U "jax[cuda12]"
python3 -m pip install -e ./EXACT_MPPI_core
```

Jetson 必须安装与 JetPack/CUDA 匹配的 JAX wheel。实机前确认：

```bash
python3 -c "import jax; print(jax.default_backend(), jax.devices())"
```

后端为 CPU 时只允许离线调试，不应接入实机。

## 编译和启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install \
  --packages-select uwb_aoa_pkg go2_exact_mppi_follow
source install/setup.bash

ros2 launch go2_exact_mppi_follow go2_exact_mppi_follow.launch.py \
  pointcloud_topic:=/local_grid_obstacle \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  one1000_topic:=/libAoa_robot_publisher
```

默认要求 JAX GPU。CPU 干运行可在 `config/go2_exact_mppi_follow.yaml` 设置 `require_jax_gpu: false`。

完整算法、参数与风险说明见 [`docs/packages/go2_exact_mppi_follow.md`](../../docs/packages/go2_exact_mppi_follow.md)。
