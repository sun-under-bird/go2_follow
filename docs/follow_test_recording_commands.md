# Go2 跟随避障测试录制命令

## 1. 使用前确认

记录脚本不会启动跟随避障，也不会向 `/cmd_vel` 发布速度。请先在其他终端正常启动
UWB、相机和跟随避障链路，再打开一个新终端执行本文命令。

确认磁盘剩余空间：

```bash
df -h /root
```

进入测试分支工作区并加载环境：

```bash
cd /root/go2_follow_reflective_noise_test
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果 `install/setup.bash` 不存在，先编译：

```bash
colcon build --symlink-install --packages-up-to go2_uwb_local_follow
source install/setup.bash
```

## 2. 推荐命令：全程记录并允许按键标记

普通跟随、转向、停车和避障问题使用 `core` 档位：

```bash
./scripts/record_follow_test.sh \
  --name follow_test \
  --profile core
```

脚本启动后会持续记录。发现问题时直接按对应按键，测试完成后按 `q`：

| 按键 | 事件标记 |
| --- | --- |
| `f` | 异常向前 |
| `b` | 异常后退 |
| `t` | 转向异常 |
| `s` | 异常停车或停止后不恢复 |
| `o` | 假障碍或误急停 |
| `j` | 机身抖动 |
| `u` | UWB 异常或丢数 |
| `n` | 输入自定义备注 |
| `h` | 重新显示按键帮助 |
| `q` | 结束录制并生成摘要 |

按键只插入事件时间，不会截断数据。脚本会保存从启动到按 `q` 之间的全部数据，包括
事件发生前和事件发生后的内容。问题发生后建议继续运行 10～20 秒再结束，以记录恢复过程。

## 3. 录制一小时

需要在一小时内随时按键标记问题时，运行下面的命令，并在约一小时后人工按 `q`：

```bash
./scripts/record_follow_test.sh \
  --name one_hour_follow \
  --profile core
```

不需要按键标记、到一小时自动停止：

```bash
./scripts/record_follow_test.sh \
  --name one_hour_follow \
  --profile core \
  --duration 3600 \
  --non-interactive
```

`core` 档位一小时常见约 15～60 GB，复杂点云情况下可能接近 130 GB。建议开始前至少
预留 100 GB；测试期间脚本不会自动删除任何旧记录。

## 4. 反光、视差和假障碍专项录制

需要分析左右原图和视差时使用 `vision`：

```bash
./scripts/record_follow_test.sh \
  --name reflective_obstacle \
  --profile vision
```

`vision` 档位在 640×480、30 Hz 下，一小时可能占用约 200～330 GB。它只适合短时间
复现反光或深度噪点，不建议默认连续录制一小时。

磁盘空间不足时可启用单线程 zstd 文件压缩：

```bash
./scripts/record_follow_test.sh \
  --name reflective_obstacle \
  --profile vision \
  --compress
```

压缩会增加 CPU 占用。分析实时控制卡顿时，优先使用不压缩的 `core` 档位。

## 5. 先录制五分钟估算容量

不同相机频率和场景的点云大小不同，可先自动录制 300 秒：

```bash
./scripts/record_follow_test.sh \
  --name capacity_check_5min \
  --profile core \
  --duration 300 \
  --non-interactive
```

查看最新记录大小：

```bash
du -sh test_records/* | tail
```

五分钟目录大小乘以 12，即为当前实机条件下一小时的大致占用。

## 6. 输出目录与重复录制

默认输出到：

```text
/root/go2_follow_reflective_noise_test/test_records/
```

每次启动都会创建带时间戳的新目录，例如：

```text
test_records/20260911_173000_follow_test/
test_records/20260911_184500_follow_test/
```

重复启动不会覆盖之前的数据；即使同一秒启动同名测试，脚本也会追加进程号。每个 rosbag
文件达到 2 GiB 后会自动切分，但整个测试仍保存在同一个目录。

指定其他磁盘作为输出位置：

```bash
./scripts/record_follow_test.sh \
  --name follow_test \
  --profile core \
  --output-root /mnt/test_disk/go2_records
```

## 7. 录制结果检查

脚本结束时会在终端输出本次测试目录。目录中最重要的内容是：

```text
rosbag2/                 全部 ROS 2 数据
events.tsv               按键事件的精确时间
问题记录.md              自动生成的中文摘要
config_snapshot/         配置副本
snapshots/               节点、参数和系统状态
runtime_resources.tsv    每秒系统资源数据
working_tree.diff        本次测试对应的未提交代码差异
```

查看 rosbag 的话题、消息数和持续时间：

```bash
ros2 bag info /完整路径/到/本次记录/rosbag2
```

回放前不要连接真实底盘速度接收端。建议使用隔离的 ROS 域：

```bash
export ROS_DOMAIN_ID=230
ros2 bag play /完整路径/到/本次记录/rosbag2
```

测试结束后，将脚本生成的整个时间戳目录交给研发人员，不要只发送数据库文件。
