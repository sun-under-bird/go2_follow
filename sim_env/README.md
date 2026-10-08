# Go2 仿真开发环境：Ubuntu 22.04 / ROS 2 Humble / MuJoCo

**另一台电脑从零安装请按 [异机从零复现指南](异机从零复现指南.md) 操作**，包含 WSL、依赖安装、网页启动、八场检查、回放、关闭与故障恢复。本文的磁盘、显卡和迁移信息是原电脑记录，不是新电脑必须采用的配置。下面 Windows 启动命令均从 Git 仓库根目录执行。

本目录保存安装脚本、版本锁定、启动入口和验证记录。[平地跟随实验台](follow_demo/README.md)当前默认身份为 **rate / trail / camera / steady / process**，相机默认水平，人的目标速度为 **0.8 m/s**。

2026-10-08速度修复加入人体历史目标区域、MPPI速度/停车评分、共享矩形计算及仿真中高速执行校准，并修复控制时钟与里程计异步到达造成的误停车。相机俯角可显式对照，完整场景和失败边界见[速度跟随修复与验收](artifacts/速度跟随修复与验收_20261008.md)。2026-10-05的五场通过属于旧源码与0.5 m/s目标速度，不能替代当前结果。

当前32 m轨迹历史版本、相机显式向下25°的八场复测为 **6/8通过**，286项逻辑检查通过。开阔稳态实测0.785 m/s；方形回环完成两圈。交错回环仍掉队且少最后一道回环门，双长墙回环完成两圈但最大间距3.68 m超过3.5 m门槛。[当前离线回放](artifacts/navigation-speed-history32-20261008-验收回放.html)包含真实失败，不能称为全部解决。

当前包络0.70×0.32 m矩形，前进/转向请求上限0.8 m/s、±1.0 rad/s，保留安全停人看向。仿真使用理想深度和真值里程计；尚未完成VIO、RK3588或实机验证。[实验台README](follow_demo/README.md)提供启动、验收和关闭步骤，[原生评分器说明](native/README.md)解释运行依赖。

## 1. 环境位置与版本

| 项目 | 配置 |
|---|---|
| Windows 发行版名称 | `Ubuntu-22.04`，WSL 2 |
| Linux 系统 | Ubuntu 22.04.5 LTS / Jammy，x86_64 |
| 虚拟磁盘目录 | `D:\WSL\Ubuntu-22.04` |
| Linux 用户 | `chy`，使用本机 WSL 登录；sudo 免密码，不启用 SSH |
| Linux 项目目录 | `/home/chy/go2_sim`，编译与模型运行都放在 Linux 文件系统 |
| ROS | ROS 2 Humble，RViz、Nav2/MPPI（本机已安装 1.1.20）、双目图像处理、rosbag2 MCAP |
| 物理仿真 | MuJoCo 3.3.6 / Unitree MuJoCo |
| 步态推理依赖 | ONNX Runtime 1.23.2，CPU 推理 |
| Python | Ubuntu Python 3.10，独立 venv，NumPy 1.26.4 |
| 图形 | WSLg + Windows 显卡驱动；已检查 RTX 4060 的 D3D12/OpenGL 加速 |
| 仿真通信 | 原生入口 DDS 域 1，跟随实验台域 42；CycloneDDS XML 显式限定网卡 `lo` |

不在 WSL 中安装 Linux NVIDIA 内核驱动。图形通过 Windows 驱动和 WSLg 提供；离屏验收使用 OSMesa，以便没有桌面窗口时也能复现。

Windows 的 `.wslconfig` 原有 `networkingMode=mirrored` 保留。Docker Desktop 的发行版保留。

## 2. 日常启动

在 **Windows PowerShell** 进入 Linux：

```powershell
wsl -d Ubuntu-22.04 -u chy
```

在 **Ubuntu 终端** 加载环境：

```bash
source ~/go2_sim/setup.bash
# 也可以在交互式 Bash 中使用别名：go2env
```

常用入口：

| 命令 | 用途 |
|---|---|
| `go2-sim` | 打开 Go2 MuJoCo 物理仿真窗口 |
| `go2-policy 30 0 0 0` | 另开 Ubuntu 终端，运行上游策略的站立指令 |
| `go2-policy --teleop` | 使用上游策略键盘控制，键位以程序提示为准 |
| `go2-capo` | 启动 LowState 适配器和 CAPO 里程计 |
| `go2-rviz` | 独立查看 Go2 URDF 和关节滑条 |
| `go2-check` | 重跑环境验收并生成 JSON、模型图片 |
| `go2-check-native` | 打开仿真窗口，重跑约 25 秒的 SDK → ROS → CAPO 链路检查 |
| `go2-follow` | 启动本机浏览器跟随实验台，使用独立 DDS 域 42 |

`go2-rviz` 是 URDF 检查入口，滑条控制的是显示姿态，不会驱动 MuJoCo。MuJoCo 使用 MJCF 物理模型，ROS 显示使用仓库 URDF；两者不是自动同步的同一个模型文件。

先启动 `go2-sim`，再在另一个终端加载同一环境并启动策略。没有控制器时，机器人会受重力自由运动，这不是安装故障。关闭策略前先结束当前演示；仅停止策略会留下模拟器最后一次收到的低层命令。

本次安装了 `go2_twist_bridge` 源码依赖，但默认入口不启动它。该接口发送的是实机 Sport 请求。原生 `go2-sim` / `go2-policy` 入口仍没有 ROS `/cmd_vel` 接口；新增 `go2-follow` 已通过 Python 适配层接入 `/cmd_vel`，复用锁定的 MuJoCo 模型和 ONNX 步态策略。两套入口独立运行，详细边界见跟随实验台文档。

跟随实验台也可直接从 Windows PowerShell 启动和停止：

```powershell
.\sim_env\Start-FollowDemo.ps1
# 验收结束后执行；仅关闭浏览器不会结束仿真。
.\sim_env\Stop-FollowDemo.ps1
# 当前默认身份也可显式写出，避免与旧实例混用：
.\sim_env\Start-FollowDemo.ps1 -ExecutionMode rate -PlanningMode trail -ObservationMode camera -ExecutorWakeMode steady -SearchExecutionMode process -CameraPitchDeg 0
# 向下25度安装试验；先停止现有实例，模式和安装参数在运行中不能切换：
.\sim_env\Stop-FollowDemo.ps1
.\sim_env\Start-FollowDemo.ps1 -CameraPitchDeg 25
```

停止脚本核实 HTTP、实验台 Python 进程和专属 Nav2 控制器/生命周期管理器均已结束，不停止其他任务或整台 WSL。[2026-10-08退出与版本核实](artifacts/navigation-speed-history32-20261008-final-audit.json)保存本轮八场源码、原生库一致性及实验台子进程退出。用户按 [实验台操作说明](follow_demo/README.md) 自行验收：先选择场景并重置、等姿态就绪，再确认起始净空、开始跟随并让目标沿预置路线行走；结束后执行停止脚本。

## 3. 从零复现

在支持 WSL2/WSLg 的 Windows 11 上使用。先确保 D 盘有足够空间，建议为新环境预留至少 25 GB。无 WSL 时先在管理员 PowerShell 运行：

```powershell
wsl --install --no-distribution
# 根据提示重启 Windows，再进行下面的安装。
```

保留整个 `sim_env` 目录，进入该目录后运行：

```powershell
.\Setup-Wsl.ps1
# 修改安装位置的示例：
# .\Setup-Wsl.ps1 -InstallLocation 'E:\WSL\Ubuntu-22.04'
```

脚本只创建或继续配置 `Ubuntu-22.04`，不自动删除其他发行版。它依次：

1. 安装 WSL Ubuntu 22.04，把脚本复制到 Linux 中固定的执行目录。
2. 更新 Ubuntu、创建 `chy` 用户、配置中文注释所述的系统设置。
3. 安装 ROS 2 Humble、RViz、Nav2、双目工具与 C++ 构建依赖。
4. 安装 Python venv，拉取指定提交的仓库，构建 SDK、仿真器、策略程序和 ROS 包。
5. 配置本机启动脚本并安装跟随实验台，运行环境验收，保存日志。跟随运动检查另按实验台文档执行。

安装中断后可以重跑。已有源码提交与版本锁不一致时会停止，避免覆盖你的代码。不要一边安装一边修改 Linux 执行目录中的脚本。

国内网络默认使用中科大 Ubuntu / ROS2 镜像，APT 保留官方签名验证。ROS2 只启用二进制包索引，不需要源码包索引。可在运行 `bootstrap_ubuntu.sh` 时通过 `GO2_UBUNTU_MIRROR`、`GO2_ROS_MIRROR` 改为其他有效镜像。不要用关闭证书检查或关闭签名验证来处理网络问题。

## 4. 版本与复现边界

`locks/versions.env` 固定了仓库提交和主要运行库版本：

| 仓库 | 锁定提交 |
|---|---|
| CAPO Go2 分支 | `f1cc153384e8821be40cbc29fb54a89f4993befc` |
| CAPO main 仿真资源 | `ca12af336e3a87158b7158b6632509307e2ba669` |
| 用户 Go2 URDF/驱动 | `1519be2485f63e54349b85f6f766b574ea6ff700` |
| Unitree MuJoCo | `4134cb5dc7ff1ba7f484deda48b5274b58694519` |
| Unitree SDK2 | `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36` |
| Unitree ROS2 消息 | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |

CAPO main 的接触力、线程同步和场景补丁应用于专用仿真副本。脚本生成可移植的步态构建配置，避开上游写死的开发者主目录。用户原有工作区不作为本次构建目录。

Python 主要依赖由 `locks/python-requirements.txt` 锁定；本次导出的 `locks/python-freeze.txt` 同时固定其 Python 间接依赖，重跑时优先使用它。安装后输出完整 `python-freeze.txt`、`apt-packages.tsv`、`git-revisions.txt` 和应用补丁。APT 镜像会持续更新；此处提供可重建的安装流程和本次版本清单，不保证未来安装的每个系统包都字节相同。

安装还修正了上游 `go2_driver` CMake 中失效的 `params` 安装项：该目录在锁定提交中不存在，启动文件也未使用。修改保存为 `locks/go2-build-fix.patch`。Unitree 消息所需的 `ros-humble-rosidl-generator-dds-idl` 已显式加入依赖，避免只安装 ROS Desktop 后仍缺少接口生成器。

如需保存整台环境的精确副本，可在停止该发行版内的工作后自行导出（会额外占用较多空间）：

```powershell
wsl --terminate Ubuntu-22.04
wsl --export Ubuntu-22.04 'D:\WSL\backups\go2-jammy-configured.tar'
```

原生 SDK 和 ROS 各用自己的 DDS 库。`go2-sim`、`go2-policy` 只在各自进程设置 SDK 动态库路径，不能把 `/opt/unitree_robotics/lib` 永久加到 ROS 的 `LD_LIBRARY_PATH`。

ROS 回环网卡限制统一由 `config/cyclonedds.xml` 指定，不再叠加 `ROS_LOCALHOST_ONLY=1`。在本机 Humble/CycloneDDS 组合中同时使用这两种配置会重复选择 `lo`，导致节点创建失败。

## 5. 本机迁移记录

原发行版 `Ubuntu` 经系统信息确认是 Ubuntu 26.04，位于 `D:\WSL\Ubuntu`。已执行注销，并把 Ubuntu-22.04 设为默认发行版。

删除前备份了旧 `/home/chy/microduck_rl` 的源码，排除可重新安装的 `.venv`：

```text
D:\WSL\backups\ubuntu26-microduck_rl-sources-20260927.tar.gz
SHA256: 92A37CD55B1E582676118D882B816EF7B6C0787357ACF1A68508517ED1159C38
```

这不是旧发行版的完整备份。归档已通过 gzip 完整性检查。

注销后曾留下约 11 GB 的旧 `D:\WSL\Ubuntu\ext4.vhdx`，代理的删除请求被自动执行检查以 `blocked by policy` 拒绝。随后用户在管理员 PowerShell 中手动删除；已核实旧文件不存在，Ubuntu-22.04 的虚拟磁盘及其默认发行版设置保留，Docker 发行版也保留。清理后检查时 D 盘可用空间为 32.53 GiB，迁移状态记录见 `logs/migration-final-state.json`。

## 6. 验证记录与后续范围

环境验收脚本会验证 ROS 包/自定义消息加载、cv_bridge 图像转换、ONNX 推理、Go2 模型物理步进及渲染、跨进程 ROS 通信。输出在 Linux 的 `~/go2_sim/artifacts`，本次最终结果也复制到本目录的 `artifacts` 中。

本次实际结果：

| 检查 | 结果 |
|---|---|
| 自定义 ROS 包 | 6 个包构建并安装成功 |
| ROS 包/消息、图像转换、ONNX 推理、物理步进/渲染、跨进程 ROS 通信 | 5 项全部通过，见 `artifacts/environment-check.json` |
| SDK → ROS → CAPO 实际链路 | 25 秒内接收 20327 条 LowState、4455 条适配 IMU、4455 条里程计；里程计未发现非有限数值，见 `artifacts/native-integration-check.json` |
| 原生仿真策略 | 站立指令进程在链路检查期间持续运行；不据此宣称行走或导航性能达标 |
| RViz / URDF | 实际启动并建立 OpenGL 4.2 上下文，关节界面收到模型，见 `logs/rviz-startup.log` |
| WSLg 显卡 | D3D12 NVIDIA GeForce RTX 4060 Laptop GPU，硬件加速启用 |
| 重启后检查 | 默认用户 `chy`、HOME 正确、systemd 为 running、启动入口与 Python 依赖可用，见 `logs/restart-validation.log` |

`artifacts/go2_model.png` 是实际 MuJoCo 渲染的模型预览。上述检查是环境与通信验收，不是里程计精度或跟随避障验收。

日志中 `lo ... disabling multicast` 表示回环网卡不支持组播，实际本机单播通信已通过检查。RViz 的 `Stereo is NOT SUPPORTED` 指显示器的立体缓冲，与双目相机算法无关。RViz 检查在 15 秒后主动发 SIGINT，关节 GUI 的退出码 `-2` 是本次主动结束造成的。URDF 根连杆惯量会触发 KDL 提示，当前模型显示和 TF 发布仍正常。

2026-10-05旧算法、234项逻辑检查、同版五场物理报告及左右看向测试见 [矩形包络与角度跟随验收](artifacts/矩形包络与角度跟随验收_20261005.md)。旧导航报告为schema 6，保存24文件SHA256和五项固定身份；当前schema 7及八场完整复测见页首链接。任务完成、流畅性及输入失效停车分别保存，单次通过不能代表重复运行成功率。

```powershell
# 拒绝复用人工正在验收的实例，自动启动、记录并关闭；每轮使用独立前缀。
.\sim_env\Check-NavigationDemo.ps1 -Scenarios open,long_wall,consecutive,corner,blocked -ReportPrefix navigation-self-rectangle
.\sim_env\Check-HeadingDemo.ps1 -ReportPrefix heading-self-rectangle
```

[2026-10-05旧版离线回放](artifacts/navigation-rectangle-verified-20261005-验收回放.html)保留历史；[2026-10-08当前八场回放](artifacts/navigation-speed-history32-20261008-验收回放.html)双击即可播放、切场景及拖动时间，不需要仿真或HTTP。新报告按实际朝向显示矩形；旧报告保留旧圆形语义。辅助障碍只解释报告，没有输入导航，最终地图不会伪装成每帧历史地图。

默认仍为DiffDrive，仅使用vx/wz。0.8/1.0是用户指定指令上限；原模型原地请求±1.0 rad/s仅约+0.443/−0.621 rad/s，不能据此宣称实际转速已经达到1.0。当前仿真真值里程计、理想深度及静态历史假设尚需替换验证；未完成CAPO/VIO、RK3588或实机验收，按要求继续暂缓低速死区及起步补偿。

## 7. 依据

新增的方形四连弯、四组交错障碍和双长墙回环，启动方法、布局与独立验收记录见 [循环场景搭建与验收](artifacts/循环场景搭建与验收_20261005.md)。原五个场景及其历史报告保留；新场景不会继承旧的通过结论。

- [Microsoft WSL 命令说明](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
- [ROS 2 Humble 的 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)
- [CAPO Go2 仓库](https://github.com/sun-under-bird/CAPO-Go2-Odometry)
- [Go2 URDF 与底盘仓库](https://github.com/sun-under-bird/go2)
