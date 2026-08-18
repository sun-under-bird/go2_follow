# `uwb_aoa_pkg` 使用说明

## 包定位

`uwb_aoa_pkg` 是整个工作区的外接 UWB 数据源。它从 115200 波特率串口读取 Ubitraq C5 数据包，再把原始距离、方位角和时间送入厂家 `algo_uwb_aoa_merge`，发布 `LibAoaRobotMsg`。

当前 UWB 模块按机器人前向对齐安装。驱动自身默认 frame 是 `uwb_link`，完整 MPPI launch 会覆盖为 `base_footprint`。如果实际安装方向或位置有偏差，应修改厂家算法输入配置或建立独立传感器 frame 和真实 TF，不能只修改 `frame_id`。

## 文件与组件

| 文件 | 作用 |
| --- | --- |
| `msg/LibAoaRobotMsg.msg` | UWB 定位、原始测量、质量和设备字段定义 |
| `src/libAoa_robot_example.cpp` | 串口配置、厂家算法调用、限频和 ROS 发布 |
| `src/uart_stack.c` | `0x55 0xAA` 帧头、长度、CRC 和 C4/C5 数据包解析 |
| `include/uwb_aoa_pkg/uwb_robot_algo.h` | 厂家算法接口 |
| `lib/uwb_robot_algo.a` | ARM64 厂家滤波静态库 |

驱动已回退为此前实机可用的 ARM64 厂家库版本。x86_64 只构建消息接口，`libAoa_robot_example` 必须在 ARM64 Go2 容器中编译。

## 节点接口

节点：`/libAoa_robot_publisher`

发布：

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/libAoa_robot_publisher` | `uwb_aoa_pkg/msg/LibAoaRobotMsg` | depth 10, reliable | 同时兼容仓库中的可靠订阅与 best-effort 订阅 |

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `frame_id` | `uwb_link` | 消息坐标系；完整 MPPI launch 会覆盖为 `base_footprint` |
| `publish_rate_hz` | `10.0` | ROS 发布频率上限，串口仍持续解析 |

串口路径是 executable 的第一个位置参数。驱动自身缺省为：

```text
/dev/ttyUSB0
```

完整 MPPI launch 默认传入当前 FTDI 设备的稳定 `by-id` 路径，也可以显式覆盖为 `/dev/ttyUSB1`。

## 坐标换算

当前已经过实测的换算为：

```text
raw C5 distance/angle/pitch + monotonic time
  -> algo_uwb_aoa_merge
  -> r/a/x/y/state
```

上层控制包不再额外做低通、跳变速度限制、状态/置信度或超时停车，直接使用厂家算法输出坐标。

## 消息字段

| 字段 | 单位/语义 |
| --- | --- |
| `header` | ROS 时间戳与 `base_footprint` frame |
| `r`, `a` | 换算后的距离 m、方位 rad |
| `x`, `y` | 换算后的二维坐标 m |
| `state` | 厂家算法状态，非负通常表示可用，负值表示厂家判定异常 |
| `rssi[6]`, `pos_confidence` | 信号与位置置信度 |
| `sync_cnt`, `fob_id`, `fob_type` | 数据同步和标签信息 |
| `raw_distance`, `raw_angle`, `raw_pitch` | C5 原始距离、角度、俯仰 |
| `rx_power`, `rssi_fpp`, `rssi_np`, `rssi_ble` | 扩展无线质量数据 |

## 编译与运行

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg
source install/setup.bash

ros2 run uwb_aoa_pkg libAoa_robot_example \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0 \
  --ros-args \
  -p frame_id:=base_footprint \
  -p publish_rate_hz:=10.0
```

通常不需要单独运行上述命令，`go2_uwb_mppi_follow/uwb_mppi_nav2.launch.py` 默认会启动该驱动。只有单独诊断串口时才手动启动，并确保同一时间只有一个进程占用设备。

推荐通过用户组或 udev 规则授予串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后确认：

```bash
ls -l /dev/serial/by-id/
ros2 topic hz /libAoa_robot_publisher
ros2 topic echo /libAoa_robot_publisher --once
```

不要长期使用 `sudo chmod 666`，权限会在设备重插后失效，而且扩大设备访问范围。

## 串口行为

```text
稳定 by-id 串口
  -> 阻塞式逐字节读取
  -> UART 状态机与 CRC 校验
  -> 厂家 algo_uwb_aoa_merge
  -> publish_rate_hz 限频
  -> /libAoa_robot_publisher
```

- 启动时设备不存在或串口配置失败：节点报告 fatal，本次进程不会自动重连。
- USB 断开后需要停止节点、确认新设备路径并重新启动。
- 完整 MPPI launch 应显式传入稳定 `by-id` 路径或实际 `/dev/ttyUSB1`。

## 常见故障

- `无法打开 UWB 串口`：检查容器是否映射 `by-id` 对应设备、`dialout` 权限和其他占用进程。
- 串口已连接但无 C5：检查标签实体按钮、标签供电、接收器供电和 USB 扩展坞电流。
- 有数据但距离跳变：记录 `raw_distance`、`raw_angle`、RSSI、`sync_cnt` 和厂家输出状态；当前上层会直接使用厂家输出，因此先保持底盘不消费 `/cmd_vel` 再分析。
- 左右方向反了或存在固定角度偏差：需要调整 `libAoa_robot_example.cpp` 中厂家算法的 `direction/config_rad`，或建立真实 UWB TF 后重新编译。
- 更换安装位置：如果偏移不可忽略，应使用独立 UWB frame 和真实 TF，而不是继续标记 `base_footprint`。
