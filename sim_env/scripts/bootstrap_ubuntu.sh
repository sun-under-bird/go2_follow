#!/usr/bin/env bash
# 配置新建的 Ubuntu 22.04，安装 ROS2 与仿真基础依赖；以 root 执行。
set -Eeuo pipefail
trap 'echo "基础配置失败：第 $LINENO 行" >&2' ERR
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../locks/versions.env"
source /etc/os-release
[[ "$VERSION_ID" == 22.04 && "$(id -u)" == 0 ]] || { echo '必须在 Ubuntu 22.04 中以 root 运行'; exit 1; }
export DEBIAN_FRONTEND=noninteractive
export TZ=Asia/Shanghai

# 当前网络使用中科大镜像下载 Ubuntu 包；APT 仍核验 Ubuntu 官方签名。
# 可用 GO2_UBUNTU_MIRROR 覆盖为官方源或其他同步镜像。
export GO2_UBUNTU_MIRROR="${GO2_UBUNTU_MIRROR:-https://mirrors.ustc.edu.cn/ubuntu}"
[[ -f /etc/apt/sources.list.go2-original ]] || cp /etc/apt/sources.list /etc/apt/sources.list.go2-original
python3 - <<'PY'
import os
from pathlib import Path
path = Path('/etc/apt/sources.list')
text = Path('/etc/apt/sources.list.go2-original').read_text()
for original in ('http://archive.ubuntu.com/ubuntu', 'http://security.ubuntu.com/ubuntu',
                 'https://archive.ubuntu.com/ubuntu', 'https://security.ubuntu.com/ubuntu'):
    text = text.replace(original, os.environ['GO2_UBUNTU_MIRROR'].rstrip('/'))
path.write_text(text)
PY

# 创建本地开发用户；不启用 SSH，管理操作使用本机 WSL 与 sudo。
id chy >/dev/null 2>&1 || useradd -m -s /bin/bash -G sudo,video,render chy
printf 'chy ALL=(ALL) NOPASSWD: ALL\n' > /etc/sudoers.d/90-go2-dev
chmod 0440 /etc/sudoers.d/90-go2-dev
visudo -cf /etc/sudoers.d/90-go2-dev
cat > /etc/wsl.conf <<'EOF'
[boot]
systemd=true
[user]
default=chy
EOF

# 限制网络重试的单次等待，全部安装输出由调用方保存到日志。
cat > /etc/apt/apt.conf.d/80-go2-network <<'EOF'
Acquire::Retries "3";
Acquire::http::Timeout "30";
Acquire::https::Timeout "30";
EOF
echo '[1/4] 更新 Ubuntu 基础系统'
apt-get update
apt-get upgrade -y
apt-get install -y --no-install-recommends \
  ca-certificates curl wget gnupg locales tzdata software-properties-common \
  git build-essential cmake ninja-build pkg-config python3-pip python3-venv \
  python3-dev unzip zip rsync jq \
  libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libeigen3-dev \
  nlohmann-json3-dev libopencv-dev libgl1-mesa-dri libegl1-mesa-dev \
  libosmesa6-dev mesa-utils xauth x11-utils xvfb ffmpeg
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8
ln -sfn /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
echo Asia/Shanghai > /etc/timezone
add-apt-repository -y universe

echo '[2/4] 安装固定版本 ROS 软件源'
curl -fL --retry 3 --connect-timeout 20 \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" \
  -o /tmp/go2-ros2-apt-source.deb
dpkg -i /tmp/go2-ros2-apt-source.deb
# 使用镜像传输包，但保持 ros2-apt-source 提供的内嵌官方公钥不变。
export GO2_ROS_MIRROR="${GO2_ROS_MIRROR:-https://mirrors.ustc.edu.cn/ros2/ubuntu}"
python3 - <<'PY'
import os
from pathlib import Path
path = Path('/usr/share/ros-apt-source/ros2.sources')
lines = path.read_text().splitlines()
lines = [('URIs: ' + os.environ['GO2_ROS_MIRROR']) if line.startswith('URIs:')
         else ('Types: deb' if line.startswith('Types:') else line) for line in lines]
path.write_text('\n'.join(lines) + '\n')
PY
apt-get update
echo '[3/4] 安装 ROS2 Humble、RViz、导航、双目与构建工具'
apt-get install -y --no-install-recommends \
  ros-humble-desktop ros-dev-tools ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-stereo-image-proc \
  ros-humble-image-pipeline ros-humble-image-transport-plugins \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-rosbag2-storage-mcap \
  ros-humble-pcl-ros ros-humble-pcl-conversions python3-colcon-common-extensions
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init

echo '[4/4] 创建隔离工作目录并保存系统版本'
install -d -o chy -g chy /home/chy/go2_sim /home/chy/go2_sim/logs /home/chy/go2_sim/locks
dpkg-query -W -f='${binary:Package}\t${Version}\n' > /home/chy/go2_sim/locks/apt-packages.tsv
cp "$script_dir/../locks/versions.env" /home/chy/go2_sim/locks/versions.env
chown -R chy:chy /home/chy/go2_sim
apt-get clean
echo 'Ubuntu / ROS2 基础配置完成'
