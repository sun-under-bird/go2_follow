# Go2 跟随速度与停车评分插件

`go2_follow_mppi_critics`适配当前WSL Ubuntu 22.04的ROS 2 Humble / Nav2 MPPI 1.1.20，是跟随实验台的小型运行依赖。它不构建用户的ROS工作区，Windows本机无需ROS。

## 为什么需要

距离路径能告诉MPPI去哪里，却不能明确要求跟上人的速度。短路径还会让优化器提前减速。`FollowSpeedCritic`将当前速度参考加入同一批候选轨迹的评分，近期权重大，允许预测后段为转弯或末端减速。

原MPPI可以设想后续继续转向，最终保护却检查当前运动保持一段时间后能否停车。两种预测不同，容易出现“优化器不停要求高速、执行层不断截停”。`BrakingCritic`提前检查同一停车条件。最终保护仍重新检查实际输出：候选加权、优化器滤波、时延和执行响应都可能改变可行性。

`src/braking_geometry.hpp`同时供C++评分器及Python的`native_braking.py`使用，采用0.70×0.32 m矩形、闭栅格接触、连续旋转误差界及0.04 s停车积分。原Python算法保留为独立对照。库缺失时逻辑检查可用原算法，但原生等价检查会明确跳过；正常演示由安装脚本先建立该库。

## 输入和边界

| 输入 | 内容 |
|---|---|
| `/follow_demo/reference_speed` | `TwistStamped`，`base_footprint`，前进0～0.8 m/s，0.3 s有效期 |
| `/follow_demo/local_geometry` | `OccupancyGrid`，`odom`；只有0是自由，其余及窗口外拒绝 |
| `/follow_demo/braking_limits` | `Float64MultiArray`；顺序：仿真时刻s、保持时间s、线制动m/s²、角制动rad/s² |
| MPPI内部状态 | 当前机身位姿、实测前速/侧速/转速及同一候选的控制 |

最终保护取候选与实测正前速的较大值，检查请求转速、实测转速及二者中值。名义线制动0.45 m/s²、角制动1 rad/s²。静态地图已按采集时刻TF补偿，执行保持0.25 s，图像年龄不再重复计入执行延迟；深度0.9 s健康门槛、未知禁行独立有效。实机须测量执行延迟、制动和过冲。

地图订阅不读取MuJoCo模型、障碍真值或人的预置路线。候选评分没有覆盖动态侵入、全部执行不确定性和实机误差。

原`seedable`接口核对名义停车末端能否经矩形扫掠接入八朝向搜索图，按离当前朝向最近的顺序尝试全部八个朝向，与原Python接入规则一致。新`continuable`接口还检查接入后0.6 m已知直行出口，供评分器优先避开只能接入一两格的盲区口袋。它是软代价的充分条件，不增加碰撞包络、不证明完整绕行已找到，也不能据失败宣称所有曲线都不可行。未知和真实碰撞的最终执行规则仍独立生效。

评分后的候选加权和平滑仍可能失去该出口，因此末级对正向跟随命令的名义停车末端再检查同一条件；零命令保留实际制动，观察动作沿用原几何准入。前进被这个条件限制、且实测前速低于0.12 m/s时，搜索器主动选择观察任务。0.6 m是本轮有限视野续行参数，并非额外机身安全距离或实机标定值。

## 安装与校验

Windows PowerShell：

```powershell
wsl.exe -d Ubuntu-22.04 -u chy --exec bash '/mnt/c/Users/chy/Documents/ChatGPT/go2_slam 2/sim_env/scripts/install_follow_demo.sh'
```

安装到`~/go2_sim/follow_native`，构建缓存位于`~/go2_sim/follow_native_build`；源码摘要不变时不重建。环境脚本加入库路径及插件索引。xtensor/xsimd宏须与宿主MPPI一致，否则可能产生张量分配器ABI不一致。

在Ubuntu终端检查实际二进制：

```bash
source ~/go2_sim/follow_demo/setup.bash
cd ~/go2_sim
python -m unittest follow_demo.tests.test_native_braking follow_demo.tests.test_execution_guard
```

等价检查包含600条随机停车轨迹、400条折线路线，以及格线接触、零转速、旋转扫角、侧移和地图原地更新。它验证计算口径，不能代替物理场景验收。物理报告还保存安装源码摘要与实际`.so`摘要，避免只核对Python文件。

采样、加权更新与输出滤波契约依据锁定版本的[上游优化器源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp)。
