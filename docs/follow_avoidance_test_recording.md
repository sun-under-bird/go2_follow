# 跟随避障实机测试记录说明

## 目的

`scripts/record_follow_test.sh` 用于把问题发生前后的 UWB、里程计、速度决策、障碍点云、
TF 和诊断信息记录到同一个 rosbag。测试人员按键标记问题后，事件也会写入 rosbag，研发
人员可以按时间快速定位“异常向前、转向异常、误停车、反光假障碍、抖动或 UWB 丢数”。

记录脚本只订阅话题，不启动跟随节点，也不向 `/cmd_vel` 发布速度。唯一新增发布的话题是
`/go2_uwb_local_follow/test_event`，消息类型为 `std_msgs/msg/String`。

## 使用步骤

先正常启动 UWB 和跟随避障链路。在另一个终端运行：

```bash
cd /root/go2_follow_reflective_noise_test
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/record_follow_test.sh --name rear_crossing
```

记录过程中直接按键：

| 按键 | 问题 |
| --- | --- |
| `f` | 异常向前 |
| `b` | 异常后退 |
| `t` | 转向异常 |
| `s` | 异常停车或停止后不恢复 |
| `o` | 假障碍或误急停 |
| `j` | 机身抖动 |
| `u` | UWB 异常或丢数 |
| `n` | 输入自定义备注 |
| `q` | 正常结束并生成摘要 |

结束后把终端显示的整个测试目录交给研发人员。目录中包括：

```text
rosbag2/                 ROS 2 数据包
events.tsv               精确事件时间
问题记录.md              自动生成的测试摘要，可继续人工补充
config_snapshot/         本次运行使用的仓库配置副本
snapshots/start/         开始时节点、话题、参数和系统状态
snapshots/end/           结束时节点、话题、参数和系统状态
runtime_resources.tsv    每秒负载、可用内存和 rosbag 大小
working_tree.diff        未提交代码差异
rosbag_record.log        rosbag 自身日志
```

## 记录档位

默认 `core` 档位已经足够分析跟随和避障决策：

```bash
./scripts/record_follow_test.sh --name normal_follow --profile core
```

复现反光、深度噪点或视差异常时使用 `vision`，它会额外记录左右红外原图和视差：

```bash
./scripts/record_follow_test.sh --name reflective_floor --profile vision
```

`vision` 的磁盘写入量明显高于 `core`。开始前应使用 `df -h` 确认空间，并尽量缩短复现
时间。脚本每 2 GiB 自动切分 rosbag 文件，避免单文件过大。

如果磁盘空间紧张，可以启用单线程 zstd 文件压缩：

```bash
./scripts/record_follow_test.sh --name reflective_floor --profile vision --compress
```

压缩会增加 CPU 使用率；评估实时控制问题时优先使用不压缩的 `core` 档位。

无人值守记录 60 秒：

```bash
./scripts/record_follow_test.sh \
  --name automatic_60s \
  --duration 60 \
  --non-interactive
```

## 数据检查

查看数据包摘要：

```bash
ros2 bag info /path/to/test_record/rosbag2
```

回放前不要连接真实底盘速度接收端。建议在隔离的 `ROS_DOMAIN_ID` 下分析：

```bash
export ROS_DOMAIN_ID=230
ros2 bag play /path/to/test_record/rosbag2
```

优先根据 `events.tsv` 或 `/go2_uwb_local_follow/test_event` 的时间检查以下对应关系：

1. `raw_angle` 与融合目标 `x/y` 是否同时变化。
2. `/go2_uwb_local_follow/nominal_cmd` 是否符合 UWB 目标。
3. `planned_cmd` 与 `final_cmd` 是否因障碍或加速度限制改变。
4. `/odom_leg` 实际速度是否跟随最终下发速度。
5. 障碍诊断、滚动地图诊断是否在事件附近出现点数或状态突变。
