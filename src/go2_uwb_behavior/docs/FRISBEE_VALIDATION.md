# 多 UWB 飞盘衔回：软件验证与交付记录

## 1. 工作树与范围

- 工作树：`C:\Users\chy\Documents\ChatGPT\go2_slam 2\go2_follow_develop_20261009`。
- 分支：`feature/uwb-frisbee-retrieve`，从 `origin/develop` 基线创建。
- 基线：`1c18eec883e94417c01abfa3c7b4ef216ac5d869`。
- 本次功能交付保存在上述功能分支；未合并至 `develop`，原开发目录未修改。
- 功能覆盖：按 ID 分发数据、一次性指定 ID/距离接近、停稳及异常结果、模拟拾取握手。
- 厂家融合算法及静态库、MPPI 算法和碰撞检测、三个旧 Action 定义均未修改。

消息、Action 全字段和错误码、参数、ID2/ID1 命令、行为树握手、RK3588 启动与实机步骤见
[接口与运行文档](FRISBEE_RETRIEVE_API.md)。

## 2. 构建环境与结果

本机 WSL `Ubuntu-22.04`，ROS 2 Humble，x86_64，GCC 11、Python 3.10。
构建命令：

```bash
cd "/mnt/c/Users/chy/Documents/ChatGPT/go2_slam 2/go2_follow_develop_20261009"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_behavior \
  --parallel-workers 2 --cmake-args -DBUILD_UWB_VENDOR_DRIVER=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 scripts/run_frisbee_tests.py --phase all
```

三个包编译已通过，最终完整构建耗时 3 分 8 秒，记录为
`log/frisbee_release_build.log`。
本机 `/mnt/c` 文件时间戳与 WSL 时钟有偏差，make 出现约 0.2～5.1 秒的
Clock skew 警告，没有编译或链接错误；核心控制器还执行过强制重新编译，记录为
`log/frisbee_verified_controller_build.log`。最后全量检查执行工作树构建产物，
结果以本文件下一节及 `log/frisbee_release_tests.log` 为准。

`BUILD_UWB_VENDOR_DRIVER=OFF` 仅跳过 x86 无法链接的 ARM64 厂家库，
消息和串口核心仍编译；串口测试使用假融合库。
不能由此声称 ARM64 厂家融合或实机运动验收通过。

## 3. 软件测试结果

最终全量测试于 2026-10-09 在本机 WSL 完成，耗时 4 分 28 秒：

- CTest：UWB 包 2/2、局部跟随包 18/18、行为包 15/15，合计 35 个测试任务全部通过。
- `colcon test-result`：385 项报告记录，0 errors、0 failures、36 skipped。
  这些记录包含测试用例及静态检查记录，不等于 385 个独立运动场景。
- 36 个跳过项来自两个包的 Cppcheck：本机版本 2.7 被 ament 因已知性能问题自动跳过；
  **Cppcheck 深度静态分析未执行**，不能把它计为实际检查通过。
- 新 Action 的 11 个集成用例全部通过；实际 MPPI 闭环及碰撞阻断通过。
- 原 Follow/RandomRoam 生命周期、Orbit 互斥与取消、导航目标及局部避障回归全部通过。
- 两个根目录 Python 测试辅助脚本的 flake8/pep257 单独检查通过，
  记录为 `log/frisbee_release_helper_lint.log`；`git diff --check` 通过。
- 没有尚未解决的软件测试失败；旧失败日志仅保留环境问题的排查过程。

测试进程组 PGID=407 已全部停止；交付前另行检查，没有此次启动的 ROS 节点、
测试驱动或 DDS 诊断探针残留。没有停止用户的其他任务。

| 检查范围 | 验证内容 |
|---|---|
| 数据链 | ID1/ID2/ID3 各自坐标、独立源时间与接收时效、ID2 不刷新旧人员话题 |
| 驱动与串口 | 背靠背 ID1/ID2 不抢占限频，串口断开和重连；厂家融合调用保持原样 |
| 到达核心 | 新样本连续达标、重复帧不计时、时间回退/间隔超限重置、安全间距、可选朝向 |
| 新 Action | A～G、动态距离切换、四个 Action 双向互斥、取消/STOP 优先级、STOP 锁存 |
| 异常与停车 | 未到达/仍运动不成功、停车后距离复检、标签和非 UWB 断流、恢复、受阻、总超时 |
| 停车失败 | 冻结时间戳或运动速度证据不能产生 SUCCESS；取消停车失败也返回 STOP_UNCONFIRMED |
| 移动人员 | 实际里程计持续进展时，相对距离不下降不会被直接判为无进展 |
| 拾取握手 H | 模拟拾取失败不请求 ID1；仅 ROS SUCCEEDED 且业务 SUCCESS 才执行下一步 |
| 实际 MPPI | 平面运动积分闭环完成 ID2 接近和停稳；足迹内障碍触发受阻、最终速度为零 |
| 原有回归 | Follow/RandomRoam 生命周期，Orbit 互斥与取消，纯导航目标服务，滚动地图、动态转向、MPPI、紧急恢复 |
| 静态检查 | 包内原有 ament 格式、Python/C++ 风格与许可检查，以及新增测试辅助脚本检查 |

A～H 的断言分别在 `test_approach_action_launch.py`、`test_approach_mppi_launch.py`、
`test_retrieve_client.py` 中，未通过放宽生产安全门限让测试通过。

### 测试环境问题及处理

1. WSL FastDDS 可发现节点但未正常交付样本：只对测试选用本机已有 CycloneDDS。
2. CycloneDDS 默认报文下较大规划器诊断丢失：测试配置限制 UDP 报文并使用 DDS 分片；
   规划器补偿/断流回归和两个行为闭环在此配置下通过，记录见
   `log/frisbee_dds_fragment_tests.log`。
3. 本机 ROS 系统时间出现约 5 秒回退：集成测试采集时钟使用同一主机的单调时间，
   跨用例保持连续；行为任务期限、接收时效和停车确认仍按实际经过时间判断。
4. Humble YAML 中 `valid_target_ids: []` 会成为未设置类型，不能传给整数数组参数：
   配置省略此项使用代码默认空白名单，需要限制时使用 `[1, 2, 3]`。

以上测试配置不改变生产 launch 的中间件、采集时间或安全判断。
旧失败日志保留用于排查，不代表最后检查结果；单项复跑也不会删除其他项目旧报告，
应以本次全量结果为准。

## 4. 修改及新增文件

以下路径相对于工作树根目录。

| 文件 | 类型 | 用途 |
|---|---|---|
| `src/uwb/msg/UwbTarget.msg` | 新增 | 带 ID、有效性和厂家状态的目标数据 |
| `src/uwb/src/libAoa_robot_example.cpp` | 修改 | 每 ID 独立限频及重连后重置 |
| `src/uwb/CMakeLists.txt` | 修改 | 消息生成及 geometry_msgs 依赖 |
| `src/uwb/package.xml` | 修改 | 消息依赖 |
| `src/uwb/test/test_serial_reconnect.py` | 修改 | 背靠背双 ID 和重连验证 |
| `src/go2_uwb_local_follow/include/go2_uwb_local_follow/uwb_target_store.hpp` | 新增 | 独立标签快照和双时效检查 |
| `src/go2_uwb_local_follow/src/uwb_target_adapter_node.cpp` | 修改 | 多 ID 路由、人员兼容话题、逐 ID 诊断 |
| `src/go2_uwb_local_follow/config/uwb_follow_only.yaml` | 修改 | 多目标话题、人员 ID、时效及白名单示例 |
| `src/go2_uwb_local_follow/CMakeLists.txt` | 修改 | 新单元及集成测试 |
| `src/go2_uwb_local_follow/package.xml` | 修改 | 测试时钟消息依赖 |
| `src/go2_uwb_local_follow/README.md` | 修改 | 中文多标签说明 |
| `src/go2_uwb_local_follow/test/test_uwb_target_store.cpp` | 新增 | 标签隔离及时效测试 |
| `src/go2_uwb_local_follow/test/test_multi_uwb_launch.py` | 新增 | 实际配置下的多标签路由和过期诊断 |
| `src/go2_uwb_local_follow/test/test_planner_recovery_launch.py` | 修改 | 明确人员 ID、就绪握手、测试采集时钟 |
| `src/go2_uwb_behavior/action/ApproachUwb.action` | 新增 | 一次性接近接口 |
| `src/go2_uwb_behavior/include/go2_uwb_behavior/approach_core.hpp` | 新增 | 仅按新目标帧累计稳定到达证据 |
| `src/go2_uwb_behavior/src/uwb_behavior_controller_node.cpp` | 修改 | 新 Action、互斥、安全门控、严格停稳和结果复检 |
| `src/go2_uwb_behavior/config/behavior_controller.yaml` | 修改 | 新接口与停靠初始参数 |
| `src/go2_uwb_behavior/launch/behavior_follow_roam.launch.py` | 修改 | 新目标话题接入及中文节点说明 |
| `src/go2_uwb_behavior/launch/uwb_visual_bringup.launch.py` | 修改 | 新话题与现有门控参数转发、中文说明 |
| `src/go2_uwb_behavior/launch/navigation_targets.launch.py` | 修改 | 既有纯目标启动文件许可和中文节点/函数说明 |
| `src/go2_uwb_behavior/CMakeLists.txt` | 修改 | 接口生成、依赖、新测试和示例安装 |
| `src/go2_uwb_behavior/package.xml` | 修改 | uwb 消息与客户端/测试依赖 |
| `src/go2_uwb_behavior/README.md` | 修改 | 新 Action 及交付文档入口 |
| `src/go2_uwb_behavior/scripts/frisbee_retrieve_client.py` | 新增 | 双成功条件和显式模拟拾取握手示例 |
| `src/go2_uwb_behavior/test/test_approach_core.cpp` | 新增 | 到达证据单元测试 |
| `src/go2_uwb_behavior/test/test_approach_action_launch.py` | 新增 | 新 Action 和安全分支集成测试 |
| `src/go2_uwb_behavior/test/test_approach_mppi_launch.py` | 新增 | 真实 MPPI 软件闭环与碰撞阻断 |
| `src/go2_uwb_behavior/test/test_retrieve_client.py` | 新增 | 模拟拾取失败和双成功条件 |
| `src/go2_uwb_behavior/test/test_behavior_action_launch.py` | 修改 | 原有断言保留，测试采集时钟及显式客户端回收 |
| `src/go2_uwb_behavior/test/test_navigation_target_launch.py` | 修改 | 原有断言保留，许可、中文说明及测试采集时钟 |
| `src/go2_uwb_behavior/docs/FRISBEE_RETRIEVE_API.md` | 新增 | 完整接口、参数、启动、握手和实机验收步骤 |
| `src/go2_uwb_behavior/docs/FRISBEE_VALIDATION.md` | 新增 | 本交付记录 |
| `scripts/run_frisbee_tests.py` | 新增 | 隔离测试域、分阶段检查、定向进程回收和实际报告检查 |
| `scripts/ros_test_clock.py` | 新增 | 仅集成测试使用的连续采集时钟 |
| `scripts/cyclonedds_test.xml` | 新增 | 仅测试使用的报文/分片配置 |

## 5. 实机仍待验证

用户已验证基站多 ID 及对应坐标正确。本次未运行 ARM64 厂家驱动、真实相机或底盘运动。
还需 RK3588 编译、RealSense D435i 感知链联调，以及固定标签/实际飞盘的运动验收。

0.50 米仅为初始最小距离：需联合机身足迹、UWB 外参、拾取机构和实测制动距离标定。
近地面飞盘可能不进入点云高度过滤范围；不可由空点云的软件测试推断可安全拾取。
默认严格停稳可能遇到既有里程计 twist 偏置，详见接口文档的标定边界。
外部拾取、Nav2 和其他速度发布者仍需上层仲裁器完成全局底盘所有权交接。

停止测试：脚本支持 Ctrl+C，自动只清理其记录的测试进程组，并打印已全部停止。
实机联调：先 cancel/STOP 并等结果，再对本次 launch 按 Ctrl+C；不要全局结束 WSL 或 Docker。
本次助手启动的测试和诊断进程在交付前均检查退出，用户的其他任务不在清理范围。
