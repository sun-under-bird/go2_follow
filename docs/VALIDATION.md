# 构建、测试与依赖验证报告

本文记录 2026-08-14 在当前工作区完成的实际验证。它证明源码、安装、外接 UWB 协议解析和现有自动测试链成立，但不替代 ARM64 实机、相机、UWB 和底盘联调。

## 验证环境

| 项目 | 当前值 |
| --- | --- |
| 工作区 | `/home/bird/go2_follow` |
| ROS | ROS 2 Humble，`/opt/ros/humble` |
| 主机架构 | `x86_64` |
| ROS 包数量 | 10 |
| 构建方式 | `colcon build --symlink-install --cmake-clean-cache` |

## 结果摘要

| 检查 | 结果 | 说明 |
| --- | --- | --- |
| 全量构建 | 通过 | 10/10 包完成 |
| 全量测试 | 通过 | 129 tests，0 errors，0 failures，15 skipped |
| 核心算法测试 | 通过 | Dynamic 6、EXACT 7、APF 5、VFH 5 |
| C++/Python 规范 | 通过 | 已启用包的 copyright、cpplint、flake8、PEP257、uncrustify 等无失败 |
| Python 语法 | 通过 | 31 个 Python 文件均通过解析/构建 |
| launch 参数解析 | 通过 | 13 个 launch 均通过 `ros2 launch ... --show-args` |
| YAML 解析 | 通过 | 13 个 YAML 文件均可由 PyYAML 加载 |
| D435i 链路审计 | 通过 | 仅接收四个指定话题；无 V4L2、图像拆分、`image_proc` 或旧 `/stereo/points*` 引用 |
| UWB 消息接口 | 通过 | x86_64 可构建消息接口；厂家驱动仅在 ARM64 构建 |
| UWB 厂家驱动 | 待 Go2 验证 | 已回退厂家 ARM64 算法库版本，本机不能链接该静态库 |
| DWB 恢复链冒烟 | 通过 | controller/behavior active，FollowPath/BackUp 客户端和服务器各 1，Ctrl-C 全部正常退出 |

测试中的 15 个 skipped 主要来自当前 ROS 环境主动跳过已知性能问题版本的 `cppcheck`，不是测试失败。

## 复现命令

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow

colcon build --symlink-install --cmake-clean-cache \
  --event-handlers console_cohesion+

source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

实际测试汇总：

```text
Summary: 129 tests, 0 errors, 0 failures, 15 skipped
```

YAML 与 Python 检查：

```bash
python3 - <<'PY'
import ast
from pathlib import Path
import yaml

for path in Path("src").glob("**/*.py"):
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

for path in Path("src").glob("**/*.yaml"):
    with path.open(encoding="utf-8") as stream:
        yaml.safe_load(stream)
PY
```

launch 使用下面的形式逐一验证：

```bash
ros2 launch <package> <file.launch.py> --show-args
```

覆盖了 Dynamic 的 3 个 launch、EXACT、APF、VFH、`go2_uwb`、DWB 的 2 个、MPPI 的 2 个、TEB 和 `jie_deamon`，共 13 个。

## UWB ARM64 驱动

外接 UWB 已回退到此前实机可用版本：`uart_stack` 解析 C5 后调用厂家 `uwb_robot_algo.a` 中的 `algo_uwb_aoa_merge`。该静态库仅支持 ARM64，因此 x86_64 本机只构建消息接口并跳过 `libAoa_robot_example`；完整驱动必须在 Go2 ARM64 容器编译和验证。

## 已自动验证的行为

- Dynamic：原始目标解析和栅格规划。
- EXACT-MPPI：目标逻辑与点云筛选。
- APF/VFH：目标解析、点云过滤、急停、横向修正、绕行侧锁定和通道恢复。
- 各 C++ 包：可编译、安装，现有 lint 配置通过。
- 所有 launch：Python 可导入，参数声明和包内资源替换可解析。
- 速度话题静态审计：所有控制器、停车逻辑与恢复 behavior 均直接使用 `/cmd_vel`，没有中间速度转发话题。
- DWB 隔离域冒烟：`controller_server` 与 `behavior_server` 进入 active；行为树连接 `/follow_path` 和 `/backup`，状态进入 ready/tracking；关闭后无残留进程。

## 尚未验证的范围

- 本机修改推送到 ARM64 Go2 容器后的真实 UWB 串口、掉电重连和长时间稳定性。
- D435i 四个输入话题的实机 QoS、左右同步、TF、视差和局部点云质量。
- 外部 `teb_local_planner`、STVL、Go2 驱动和 EXACT-MPPI/JAX GPU 运行时。
- Go2 真实 TF、QoS、Nav2 lifecycle、碰撞失败和实际恢复运动联调。
- `/cmd_vel` 与当前 Go2 驱动的类型、QoS、坐标方向和 watchdog 契约。
- UWB、点云、odom、TF 分别断流后的旧数据复用与外部急停实测。
- rosbag 场景回归及机器人急停、遮挡、窄通道和目标丢失实测。

因此，自动测试全绿只说明当前代码基线可靠，不能等同于实机安全。上机前应完成 [优化建议](OPTIMIZATION.md) 中剩余 P0 项。
