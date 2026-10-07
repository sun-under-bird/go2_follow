#!/usr/bin/env bash
# 安装连续跟随源码及小型速度评分插件，复用发行版Nav2，不构建用户ROS工作区。
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_root="$HOME/go2_sim"
[[ -x "$sim_root/venv/bin/python" && -f "$sim_root/setup.bash" ]] || { echo '请先完成基础仿真环境安装'; exit 1; }
for binary in /opt/ros/humble/lib/nav2_controller/controller_server /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager; do
    [[ -x "$binary" ]] || { echo "缺少 Nav2 运行程序：$binary。请先按环境文档安装 ROS 2 Humble Navigation2。"; exit 1; }
done
[[ -f /opt/ros/humble/share/nav2_mppi_controller/critics.xml ]] || { echo '缺少 ros-humble-nav2-mppi-controller 运行包'; exit 1; }
mkdir -p "$sim_root/follow_demo" "$sim_root/bin"
cp -r "$script_dir/../follow_demo/." "$sim_root/follow_demo/"
# 插件在WSL构建为运行依赖，按源码摘要缓存，日常启动不重复构建。
set +u
source "$sim_root/setup.bash"
set -u
plugin_source="$script_dir/../native/go2_follow_mppi_critics"
plugin_digest="$(find "$plugin_source" -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)"
plugin_prefix="$sim_root/follow_native"
if [[ ! -f "$plugin_prefix/source.sha256" || "$(cat "$plugin_prefix/source.sha256")" != "$plugin_digest" ]]; then
    cmake -S "$plugin_source" -B "$sim_root/follow_native_build" -DCMAKE_INSTALL_PREFIX="$plugin_prefix" \
        -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
    cmake --build "$sim_root/follow_native_build" --parallel 2
    cmake --install "$sim_root/follow_native_build"
    printf '%s\n' "$plugin_digest" > "$plugin_prefix/source.sha256"
fi
cat > "$sim_root/bin/go2-follow" <<'EOF'
#!/usr/bin/env bash
# 演示使用独立 DDS 域，避免与原生仿真检查或其他开发任务串话。
source "$HOME/go2_sim/follow_demo/setup.bash"
export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
cd "$HOME/go2_sim"
exec python -m follow_demo.app "$@"
EOF
chmod +x "$sim_root/bin/go2-follow"
echo '已安装：go2-follow；浏览器地址 http://localhost:8765'
