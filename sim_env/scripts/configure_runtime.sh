#!/usr/bin/env bash
# 配置仅在本机回环网卡通信的仿真入口，不启动实机 Sport 控制接口。
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_root="$HOME/go2_sim"
mkdir -p "$sim_root"/{bin,config,tools,artifacts,logs}
cp "$script_dir"/smoke_environment.py "$sim_root/tools/"
cp "$script_dir"/smoke_integration.py "$sim_root/tools/"
cp "$script_dir"/view_go2.launch.py "$sim_root/tools/"
cp "$script_dir/../config/go2.rviz" "$sim_root/config/"
cat > "$sim_root/config/cyclonedds.xml" <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
EOF
cat > "$sim_root/setup.bash" <<'EOF'
# 在新终端执行 source ~/go2_sim/setup.bash，进入独立的本机仿真环境。
source /opt/ros/humble/setup.bash
source "$HOME/go2_sim/ros_ws/install/setup.bash"
source "$HOME/go2_sim/venv/bin/activate"
export ROS_DOMAIN_ID=1
# 回环限制统一由 CycloneDDS XML 指定；同时设置 ROS_LOCALHOST_ONLY 会重复注入 lo。
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/go2_sim/config/cyclonedds.xml"
export PATH="$HOME/go2_sim/bin:$PATH"
export PYTHONNOUSERSITE=1
# 由 WSLg 自动选择显卡；如需指定，用户可在加载环境前设置 MESA_D3D12_DEFAULT_ADAPTER_NAME。
EOF

# 原生 SDK 使用自己的 DDS 动态库；不要把此 LD_LIBRARY_PATH 加到 ROS 终端全局环境。
cat > "$sim_root/bin/go2-sim" <<'EOF'
#!/usr/bin/env bash
set -e
unset CYCLONEDDS_URI
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib"
cd "$HOME/go2_sim/third_party/unitree_mujoco/simulate/build"
exec ./unitree_mujoco "$@"
EOF
cat > "$sim_root/bin/go2-policy" <<'EOF'
#!/usr/bin/env bash
# 使用上游仿真策略，可传入 30 0 0 0（站立）或 --teleop（键盘）。
set -e
unset CYCLONEDDS_URI
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:$HOME/go2_sim/vendor/onnxruntime/lib"
if [[ $# == 0 ]]; then set -- 30 0 0 0; fi
exec "$HOME/go2_sim/build/gait_tools/policy_runner" "$@" \
  "$HOME/go2_sim/third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx"
EOF
cat > "$sim_root/bin/go2-capo" <<'EOF'
#!/usr/bin/env bash
# 只启动 LowState 适配器和 CAPO 里程计，不发布底盘命令。
source "$HOME/go2_sim/setup.bash"
exec ros2 launch fusion_estimator go2_capo.launch.py "$@"
EOF
cat > "$sim_root/bin/go2-rviz" <<'EOF'
#!/usr/bin/env bash
# 展示 URDF 与关节滑条；该入口不与实机驱动或仿真控制器连接。
source "$HOME/go2_sim/setup.bash"
exec ros2 launch "$HOME/go2_sim/tools/view_go2.launch.py"
EOF
cat > "$sim_root/bin/go2-check" <<'EOF'
#!/usr/bin/env bash
source "$HOME/go2_sim/setup.bash"
export MUJOCO_GL=osmesa
exec python "$HOME/go2_sim/tools/smoke_environment.py"
EOF
cat > "$sim_root/bin/go2-check-native" <<'EOF'
#!/usr/bin/env bash
# 打开模拟器窗口，执行约 25 秒的 SDK -> ROS -> CAPO 实际通信检查。
source "$HOME/go2_sim/setup.bash"
exec python "$HOME/go2_sim/tools/smoke_integration.py"
EOF
chmod +x "$sim_root"/bin/go2-*
python3 - <<'PY'
from pathlib import Path
import yaml
path = Path.home() / 'go2_sim/third_party/unitree_mujoco/simulate/config.yaml'
config = yaml.safe_load(path.read_text())
config.update(robot='go2', robot_scene='scene.xml', domain_id=1, interface='lo', use_joystick=0, enable_elastic_band=0)
path.write_text('# 本机 Go2 仿真：DDS 域 1，仅使用回环网卡，关闭手柄和悬挂带。\n' + yaml.safe_dump(config, sort_keys=False))
path = Path.home() / '.bashrc'
text = path.read_text()
line = "alias go2env='source ~/go2_sim/setup.bash'"
if line not in text:
    path.write_text(text + '\n# 手动进入 Go2 仿真环境，避免影响其他项目。\n' + line + '\n')
PY
echo '仿真入口已配置：source ~/go2_sim/setup.bash'
