#!/usr/bin/env bash
# 安装固定版本的 Go2 仿真栈；普通用户 chy 执行，构建目录位于 WSL 原生文件系统。
set -Eeuo pipefail
trap 'echo "Go2 栈配置失败：第 $LINENO 行" >&2' ERR
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../locks/versions.env"
[[ "$(id -un)" == chy ]] || { echo '请使用 chy 用户执行'; exit 1; }
sim_root="$HOME/go2_sim"
mkdir -p "$sim_root"/{third_party,downloads,logs,locks,ros_ws/src,bin,artifacts}

# 固定到给定提交；已存在目录只验证，不覆盖用户后来做出的源码修改。
fetch_revision() {
  local name="$1" url="$2" revision="$3"
  local destination="$sim_root/third_party/$name"
  if [[ ! -d "$destination/.git" ]]; then
    git init "$destination"
    git -C "$destination" remote add origin "$url"
  fi
  # 下载中断留下空仓库时，允许从同一锁定提交继续。
  if ! git -C "$destination" rev-parse --verify HEAD >/dev/null 2>&1; then
    git -C "$destination" -c http.version=HTTP/1.1 fetch --depth 1 origin "$revision"
    git -C "$destination" checkout --detach FETCH_HEAD
  fi
  [[ "$(git -C "$destination" rev-parse HEAD)" == "$revision" ]] || {
    echo "已有目录版本不匹配，保留现场：$destination"; return 1;
  }
}

# 仅在内容变化时替换文件，重跑安装时不触发无意义的重新构建。
copy_if_changed() {
  cmp -s "$1" "$2" || cp "$1" "$2"
}

echo '[1/6] 安装独立 Python 仿真环境'
[[ -x "$sim_root/venv/bin/python" ]] || python3 -m venv --system-site-packages "$sim_root/venv"
"$sim_root/venv/bin/python" -m pip install --upgrade pip==25.2
python_lock="$script_dir/../locks/python-requirements.txt"
[[ ! -f "$script_dir/../locks/python-freeze.txt" ]] || python_lock="$script_dir/../locks/python-freeze.txt"
"$sim_root/venv/bin/python" -m pip install -r "$python_lock"
"$sim_root/venv/bin/python" -m pip freeze --local > "$sim_root/locks/python-freeze.txt"

echo '[2/6] 获取固定提交的源码和模型'
fetch_revision unitree_sdk2 https://github.com/unitreerobotics/unitree_sdk2.git "$UNITREE_SDK2_COMMIT"
fetch_revision unitree_ros2 https://github.com/unitreerobotics/unitree_ros2.git "$UNITREE_ROS2_COMMIT"
fetch_revision unitree_mujoco https://github.com/unitreerobotics/unitree_mujoco.git "$UNITREE_MUJOCO_COMMIT"
fetch_revision capo-go2 https://github.com/sun-under-bird/CAPO-Go2-Odometry.git "$CAPO_GO2_COMMIT"
fetch_revision capo-sim-assets https://github.com/sun-under-bird/CAPO-Go2-Odometry.git "$CAPO_SIM_COMMIT"
fetch_revision go2 https://github.com/sun-under-bird/go2.git "$GO2_COMMIT"

echo '[3/6] 安装 MuJoCo C/C++ 运行库和 Unitree SDK2'
archive="$sim_root/downloads/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
if [[ ! -s "$archive" ]]; then
  curl -fL --retry 3 --connect-timeout 20 \
    "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" -o "$archive"
fi
mkdir -p "$sim_root/vendor"
[[ -d "$sim_root/vendor/mujoco-${MUJOCO_VERSION}" ]] || tar -xzf "$archive" -C "$sim_root/vendor"
sha256sum "$archive" > "$sim_root/locks/download-sha256.txt"
sha256sum "$sim_root/third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx" >> "$sim_root/locks/download-sha256.txt"
cmake -S "$sim_root/third_party/unitree_sdk2" -B "$sim_root/build/sdk2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
cmake --build "$sim_root/build/sdk2" -j 4
sudo cmake --install "$sim_root/build/sdk2"

echo '[4/6] 构建 MuJoCo 和独立仿真步态工具'
mujoco_repo="$sim_root/third_party/unitree_mujoco"
sim_assets="$sim_root/third_party/capo-sim-assets/sim_patch"
ln -sfn "$sim_root/vendor/mujoco-${MUJOCO_VERSION}" "$mujoco_repo/simulate/mujoco"
# 仅把已锁定仿真补丁应用于专用第三方副本，不修改用户的实机仓库。
copy_if_changed "$sim_assets/unitree_sdk2_bridge.h" "$mujoco_repo/simulate/src/unitree_sdk2_bridge.h"
copy_if_changed "$sim_assets/main.cc" "$mujoco_repo/simulate/src/main.cc"
copy_if_changed "$sim_assets/go2_scene/scene.xml" "$mujoco_repo/unitree_robots/go2/scene.xml"
copy_if_changed "$sim_assets/scene_terrain.xml" "$mujoco_repo/unitree_robots/go2/scene_terrain.xml"
cmake -S "$mujoco_repo/simulate" -B "$mujoco_repo/simulate/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$mujoco_repo/simulate/build" -j 4

# 上游示例写死了另一台机器的主目录，单独生成可移植的工具构建工程。
mkdir -p "$sim_root/vendor/onnxruntime/include" "$sim_root/vendor/onnxruntime/lib" "$sim_root/gait_tools"
for header in onnxruntime_c_api.h onnxruntime_cxx_api.h onnxruntime_cxx_inline.h onnxruntime_ep_c_api.h onnxruntime_float16.h; do
  curl -fL --retry 3 --connect-timeout 20 \
    "https://raw.githubusercontent.com/microsoft/onnxruntime/v1.23.2/include/onnxruntime/core/session/$header" \
    -o "$sim_root/vendor/onnxruntime/include/$header"
done
ort_lib="$($sim_root/venv/bin/python -c 'import pathlib,onnxruntime; print(next((pathlib.Path(onnxruntime.__file__).parent/"capi").glob("libonnxruntime.so.*")))')"
ln -sfn "$ort_lib" "$sim_root/vendor/onnxruntime/lib/libonnxruntime.so"
ln -sfn "$ort_lib" "$sim_root/vendor/onnxruntime/lib/libonnxruntime.so.1"
copy_if_changed "$sim_assets/example_cpp/gait_go2.cpp" "$sim_root/gait_tools/gait_go2.cpp"
copy_if_changed "$sim_assets/example_cpp/policy_runner.cpp" "$sim_root/gait_tools/policy_runner.cpp"
cat > "$sim_root/gait_tools/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(go2_sim_gait_tools LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
add_executable(gait_go2 gait_go2.cpp)
target_link_libraries(gait_go2 unitree_sdk2)
# ONNX Runtime 根路径由安装脚本传入，不依赖开发者的用户名。
add_executable(policy_runner policy_runner.cpp)
target_include_directories(policy_runner PRIVATE "${ORT_ROOT}/include")
target_link_directories(policy_runner PRIVATE "${ORT_ROOT}/lib")
set_target_properties(policy_runner PROPERTIES BUILD_RPATH "${ORT_ROOT}/lib")
target_link_libraries(policy_runner unitree_sdk2 onnxruntime)
EOF
cmake -S "$sim_root/gait_tools" -B "$sim_root/build/gait_tools" \
  -DCMAKE_BUILD_TYPE=Release -DORT_ROOT="$sim_root/vendor/onnxruntime"
cmake --build "$sim_root/build/gait_tools" -j 4

echo '[5/6] 构建 ROS2 消息、CAPO go2 分支和描述/底盘接口'
# 上游已无 params 目录且启动文件不引用它，移除失效安装项，保留实际资源目录。
python3 - <<'PY'
from pathlib import Path
path = Path.home() / 'go2_sim/third_party/go2/src/go2_driver/CMakeLists.txt'
text = path.read_text()
old = '  DIRECTORY launch params rviz'
new = '  # 只安装仓库中实际存在且使用的启动与可视化资源。\n  DIRECTORY launch rviz'
if old in text:
    path.write_text(text.replace(old, new))
elif new not in text:
    raise RuntimeError('go2_driver 的安装配置已变化，请检查后再继续')
PY
unitree_msgs="$sim_root/third_party/unitree_ros2/cyclonedds_ws/src/unitree"
for package in unitree_go unitree_api; do
  ln -sfn "$unitree_msgs/$package" "$sim_root/ros_ws/src/$package"
done
ln -sfn "$sim_root/third_party/capo-go2" "$sim_root/ros_ws/src/fusion_estimator"
for package in go2_description go2_driver go2_twist_bridge; do
  ln -sfn "$sim_root/third_party/go2/src/$package" "$sim_root/ros_ws/src/$package"
done
# ROS 环境脚本不保证支持 nounset，因此加载时临时关闭。
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$sim_root/ros_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

echo '[6/6] 保存版本与源码差异记录'
bash "$script_dir/configure_runtime.sh"
bash "$script_dir/install_follow_demo.sh"
rosdep update --rosdistro humble > "$sim_root/logs/rosdep-update.log" 2>&1
for repository in "$sim_root"/third_party/*; do
  [[ -d "$repository/.git" ]] || continue
  printf '%s %s\n' "$(basename "$repository")" "$(git -C "$repository" rev-parse HEAD)"
done > "$sim_root/locks/git-revisions.txt"
git -C "$mujoco_repo" diff > "$sim_root/locks/unitree-mujoco-applied.patch"
git -C "$sim_root/third_party/go2" diff > "$sim_root/locks/go2-build-fix.patch"
sudo dpkg-query -W -f='${binary:Package}\t${Version}\n' > "$sim_root/locks/apt-packages.tsv"
echo 'Go2 仿真依赖构建完成'
