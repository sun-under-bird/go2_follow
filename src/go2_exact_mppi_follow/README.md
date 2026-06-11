# Go2 EXACT-MPPI UWB Follow

This package implements a Go2 follow controller that follows the EXACT-MPPI strategy:
current stereo point cloud is filtered into local obstacle points, then passed directly
to `exact_mppi.mppi_jax.controller.MPPIController` without a Nav2 costmap.

## Inputs and outputs

- Input UWB target: `/libAoa_robot_publisher`
- Input stereo cloud: `/stereo/points2`
- Input odometry: `/odom`
- Output local goal: `/exact_mppi/goal_local`
- Output filtered obstacle cloud: `/exact_mppi/filtered_points`
- Output raw command: `/exact_mppi/cmd_vel_raw`
- Output final command: `/cmd_vel`

## Third-party EXACT-MPPI dependency

Install the upstream repository at the pinned commit:

```bash
mkdir -p third_party
git clone https://github.com/caseypen/EXACT-mppi.git third_party/EXACT-mppi
cd third_party/EXACT-mppi
git checkout 54dcd24ae7284f6779adb8e72cc702c3a6d65095
python -m pip install -U "jax[cuda12]"
python -m pip install -e ./EXACT_MPPI_core
```

On Jetson Orin Nano, first verify JAX sees the GPU:

```bash
python -c "import jax; print(jax.default_backend(), jax.devices())"
```

If the backend is CPU, do not start real robot tests.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uwb_aoa_pkg go2_exact_mppi_follow --symlink-install
source install/setup.bash

ros2 launch go2_exact_mppi_follow go2_exact_mppi_follow.launch.py \
  pointcloud_topic:=/stereo/points2 \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  one1000_topic:=/libAoa_robot_publisher
```

The default node requires a JAX GPU backend. For dry CPU-only debugging, set
`require_jax_gpu: false` in `config/go2_exact_mppi_follow.yaml`.
