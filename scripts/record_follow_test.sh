#!/usr/bin/env bash

# 一键记录 Go2 UWB 跟随避障实机测试数据，并用事件标记对齐问题发生时刻。
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_ROOT="${WORKSPACE_ROOT}/test_records"
PROFILE="core"
CASE_NAME="follow_test"
COMPRESSION="none"
DURATION_SEC=0
NON_INTERACTIVE=false
MARKER_TOPIC="/go2_uwb_local_follow/test_event"
MAX_BAG_SIZE_BYTES=2147483648
START_TIME_ISO=""
SESSION_DIR=""
BAG_DIR=""
BAG_PID=""
MONITOR_PID=""
CLEANED=false
EVENT_INDEX=0

# 打印命令参数和两种记录档位的用途。
print_usage()
{
  cat <<'EOF'
用法：
  scripts/record_follow_test.sh [选项]

选项：
  --name NAME          本次测试名称，默认 follow_test
  --profile PROFILE    core 或 vision，默认 core
  --output-root DIR    记录根目录，默认工作区/test_records
  --compress           使用单线程 zstd 文件压缩（会增加 CPU）
  --duration SEC       指定自动停止秒数，0 表示按 q 或 Ctrl+C 停止
  --non-interactive    不读取按键，使用 Ctrl+C 或 --duration 停止
  -h, --help           显示帮助

档位：
  core    UWB、目标、里程计、速度链路、处理后障碍点云、TF、诊断
  vision  在 core 基础上增加双目原图、相机参数和视差，磁盘占用明显增加
EOF
}

# 将测试名称转换为安全的目录名，避免空格和特殊字符影响记录路径。
sanitize_name()
{
  local raw_name="$1"
  local safe_name
  safe_name="$(printf '%s' "${raw_name}" | tr -cs '[:alnum:]_.-' '_')"
  safe_name="${safe_name#_}"
  safe_name="${safe_name%_}"
  if [[ -z "${safe_name}" ]]; then
    safe_name="follow_test"
  fi
  printf '%s' "${safe_name}"
}

# 解析记录档位、测试名称、输出目录和自动停止时间。
parse_arguments()
{
  while (($# > 0)); do
    case "$1" in
      --name)
        [[ $# -ge 2 ]] || { echo "错误：--name 缺少参数" >&2; exit 2; }
        CASE_NAME="$2"
        shift 2
        ;;
      --profile)
        [[ $# -ge 2 ]] || { echo "错误：--profile 缺少参数" >&2; exit 2; }
        PROFILE="$2"
        shift 2
        ;;
      --output-root)
        [[ $# -ge 2 ]] || { echo "错误：--output-root 缺少参数" >&2; exit 2; }
        OUTPUT_ROOT="$2"
        shift 2
        ;;
      --compress)
        COMPRESSION="file"
        shift
        ;;
      --duration)
        [[ $# -ge 2 ]] || { echo "错误：--duration 缺少参数" >&2; exit 2; }
        DURATION_SEC="$2"
        shift 2
        ;;
      --non-interactive)
        NON_INTERACTIVE=true
        shift
        ;;
      -h|--help)
        print_usage
        exit 0
        ;;
      *)
        echo "错误：未知参数 $1" >&2
        print_usage >&2
        exit 2
        ;;
    esac
  done

  if [[ "${PROFILE}" != "core" && "${PROFILE}" != "vision" ]]; then
    echo "错误：--profile 只能是 core 或 vision" >&2
    exit 2
  fi
  if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]]; then
    echo "错误：--duration 必须是非负整数秒" >&2
    exit 2
  fi
  CASE_NAME="$(sanitize_name "${CASE_NAME}")"
}

# 加载 ROS 2 和当前工作区环境，确保自定义消息类型能够被 rosbag 识别。
load_ros_environment()
{
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "错误：未找到 /opt/ros/humble/setup.bash" >&2
    exit 1
  fi
  # ROS 的环境脚本可能读取未定义变量，因此在启用 nounset 前加载。
  source /opt/ros/humble/setup.bash
  if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
  elif ! ros2 pkg prefix go2_uwb_local_follow >/dev/null 2>&1; then
    echo "错误：当前工作区尚未编译，请先执行 colcon build 并 source install/setup.bash" >&2
    exit 1
  fi
  set -u
}

# 保存节点、话题、参数、系统资源和内核日志快照，便于离线复现实机环境。
capture_snapshot()
{
  local stage="$1"
  local snapshot_dir="${SESSION_DIR}/snapshots/${stage}"
  mkdir -p "${snapshot_dir}/parameters"

  {
    date --iso-8601=ns
    uname -a
    uptime
    free -h
    df -h "${OUTPUT_ROOT}"
  } >"${snapshot_dir}/system.txt" 2>&1 || true
  ros2 node list >"${snapshot_dir}/nodes.txt" 2>&1 || true
  ros2 topic list -t >"${snapshot_dir}/topics.txt" 2>&1 || true
  ros2 service list -t >"${snapshot_dir}/services.txt" 2>&1 || true
  dmesg -T | tail -n 500 >"${snapshot_dir}/kernel_tail.txt" 2>&1 || true
  ls -l /dev/serial/by-id >"${snapshot_dir}/serial_by_id.txt" 2>&1 || true
  env | sort | grep -E '^(ROS_|RMW_|FASTRTPS_|CYCLONEDDS_)' \
    >"${snapshot_dir}/ros_environment.txt" 2>&1 || true

  while IFS= read -r node_name; do
    [[ -n "${node_name}" ]] || continue
    local parameter_file
    parameter_file="$(printf '%s' "${node_name#/}" | tr '/' '_')"
    ros2 param dump "${node_name}" \
      >"${snapshot_dir}/parameters/${parameter_file}.yaml" 2>&1 || true
  done < <(ros2 node list 2>/dev/null || true)
}

# 每秒记录负载、内存、磁盘和关键 ROS 进程资源占用。
monitor_resources()
{
  local output_file="${SESSION_DIR}/runtime_resources.tsv"
  printf 'time_iso\tepoch_sec\tload_1m\tmem_available_kb\tbag_bytes\n' >"${output_file}"
  while [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; do
    local now_iso epoch_sec load_1m mem_available bag_bytes
    now_iso="$(date --iso-8601=ns)"
    epoch_sec="$(date +%s.%N)"
    load_1m="$(awk '{print $1}' /proc/loadavg)"
    mem_available="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
    bag_bytes="$(du -sb "${BAG_DIR}" 2>/dev/null | awk '{print $1}')"
    printf '%s\t%s\t%s\t%s\t%s\n' \
      "${now_iso}" "${epoch_sec}" "${load_1m}" "${mem_available:-0}" "${bag_bytes:-0}" \
      >>"${output_file}"
    ps -eo pid,etimes,pcpu,pmem,rss,stat,comm,args --no-headers \
      | grep -E 'libAoa|uwb_|stereo|disparity|rolling_obstacle|local_velocity|realsense' \
      | grep -v grep >"${SESSION_DIR}/runtime_processes_latest.txt" || true
    sleep 1
  done
}

# 对 YAML 字符串做最小转义，使自定义备注能够安全发布到事件话题。
escape_marker_text()
{
  local text="$1"
  text="${text//$'\n'/ }"
  text="${text//$'\t'/ }"
  text="${text//\\/\\\\}"
  text="${text//\"/\\\"}"
  printf '%s' "${text}"
}

# 同时写入 TSV 和 rosbag 事件话题，让问题时刻能与传感器及速度数据精确对齐。
record_event()
{
  local event_type="$1"
  local note="${2:-}"
  local now_iso epoch_sec event_text escaped_text
  EVENT_INDEX=$((EVENT_INDEX + 1))
  now_iso="$(date --iso-8601=ns)"
  epoch_sec="$(date +%s.%N)"
  event_text="${event_type}"
  if [[ -n "${note}" ]]; then
    event_text="${event_type}: ${note}"
  fi
  note="${note//$'\n'/ }"
  note="${note//$'\t'/ }"
  printf '%s\t%s\t%s\t%s\t%s\n' \
    "${EVENT_INDEX}" "${now_iso}" "${epoch_sec}" "${event_type}" "${note}" \
    >>"${SESSION_DIR}/events.tsv"

  escaped_text="$(escape_marker_text "${event_text}")"
  timeout 3s ros2 topic pub --once "${MARKER_TOPIC}" std_msgs/msg/String \
    "{data: \"${escaped_text}\"}" >/dev/null 2>&1 || true
  printf '\n已标记 #%d：%s（%s）\n' "${EVENT_INDEX}" "${event_text}" "${now_iso}"
}

# 显示测试人员可用的快捷事件按键。
print_event_menu()
{
  cat <<'EOF'

记录已启动，发现问题时按键标记：
  f  异常向前       b  异常后退
  t  转向异常       s  异常停车/不再移动
  o  假障碍/误急停  j  机身抖动
  u  UWB异常/丢数   n  输入自定义备注
  h  重新显示帮助   q  正常结束记录

EOF
}

# 根据事件 TSV 生成可直接交回研发人员的问题记录 Markdown。
generate_summary()
{
  local end_time_iso branch_name commit_id bag_size
  end_time_iso="$(date --iso-8601=ns)"
  branch_name="$(git -C "${WORKSPACE_ROOT}" branch --show-current 2>/dev/null || true)"
  commit_id="$(git -C "${WORKSPACE_ROOT}" rev-parse HEAD 2>/dev/null || true)"
  bag_size="$(du -sh "${BAG_DIR}" 2>/dev/null | awk '{print $1}')"

  {
    echo "# 跟随避障实机测试记录"
    echo
    echo "- 测试名称：\`${CASE_NAME}\`"
    echo "- 开始时间：\`${START_TIME_ISO}\`"
    echo "- 结束时间：\`${end_time_iso}\`"
    echo "- 记录档位：\`${PROFILE}\`"
    echo "- Git 分支：\`${branch_name}\`"
    echo "- Git 提交：\`${commit_id}\`"
    echo "- rosbag 大小：\`${bag_size:-未知}\`"
    echo
    echo "## 事件标记"
    echo
    echo "| 编号 | 时间 | 类型 | 备注 |"
    echo "| ---: | --- | --- | --- |"
    tail -n +2 "${SESSION_DIR}/events.tsv" | while IFS=$'\t' read -r index time_iso _ type note; do
      note="${note//|/\\|}"
      printf '| %s | %s | %s | %s |\n' "${index}" "${time_iso}" "${type}" "${note}"
    done
    echo
    echo "## 测试人员补充"
    echo
    echo "- 测试场地："
    echo "- 光照/反光条件："
    echo "- 人员运动路线："
    echo "- 机器人实际表现："
    echo "- 是否可稳定复现："
    echo "- 其他说明："
  } >"${SESSION_DIR}/问题记录.md"
}

# 停止 rosbag、补齐结束快照和摘要；多次触发时只执行一次。
cleanup()
{
  if [[ "${CLEANED}" == true ]]; then
    return
  fi
  CLEANED=true
  set +e

  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    record_event "记录结束" "测试人员结束记录"
    kill -INT "${BAG_PID}" 2>/dev/null
    for _ in $(seq 1 50); do
      kill -0 "${BAG_PID}" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "${BAG_PID}" 2>/dev/null; then
      echo "警告：rosbag 未及时退出，发送 TERM" >&2
      kill -TERM "${BAG_PID}" 2>/dev/null
    fi
    wait "${BAG_PID}" 2>/dev/null
  fi
  if [[ -n "${MONITOR_PID}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    kill -TERM "${MONITOR_PID}" 2>/dev/null
    wait "${MONITOR_PID}" 2>/dev/null
  fi

  capture_snapshot "end"
  generate_summary
  echo
  echo "记录完成：${SESSION_DIR}"
  echo "问题摘要：${SESSION_DIR}/问题记录.md"
}

# 交互读取快捷键，把测试人员观察到的问题写成时间对齐事件。
interactive_loop()
{
  print_event_menu
  while kill -0 "${BAG_PID}" 2>/dev/null; do
    local key note
    if ! IFS= read -r -s -n 1 key; then
      break
    fi
    case "${key}" in
      f|F) record_event "异常向前" ;;
      b|B) record_event "异常后退" ;;
      t|T) record_event "转向异常" ;;
      s|S) record_event "异常停车" ;;
      o|O) record_event "假障碍或误急停" ;;
      j|J) record_event "机身抖动" ;;
      u|U) record_event "UWB异常" ;;
      n|N)
        printf '\n请输入备注后回车：'
        IFS= read -r note || note=""
        record_event "自定义备注" "${note}"
        ;;
      h|H) print_event_menu ;;
      q|Q) break ;;
      *) ;;
    esac
  done
}

# 组装记录话题并启动 rosbag、资源监控及事件交互主流程。
main()
{
  parse_arguments "$@"
  load_ros_environment

  command -v ros2 >/dev/null 2>&1 || { echo "错误：找不到 ros2" >&2; exit 1; }
  mkdir -p "${OUTPUT_ROOT}"
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M%S)"
  SESSION_DIR="${OUTPUT_ROOT}/${timestamp}_${CASE_NAME}"
  if [[ -e "${SESSION_DIR}" ]]; then
    SESSION_DIR="${SESSION_DIR}_$$"
  fi
  BAG_DIR="${SESSION_DIR}/rosbag2"
  mkdir -p "${SESSION_DIR}/snapshots"
  START_TIME_ISO="$(date --iso-8601=ns)"
  printf 'index\ttime_iso\tepoch_sec\ttype\tnote\n' >"${SESSION_DIR}/events.tsv"

  local -a topics=(
    /libAoa_robot_publisher
    /uwb/target_point
    /uwb/target_adapter_diagnostics
    /go2_uwb_local_follow/nominal_cmd
    /go2_uwb_local_follow/follow_diagnostics
    /odom_leg
    /local_grid_obstacle
    /local_depth_observation
    /local_rolling_obstacle
    /stereo/obstacle_diagnostics
    /go2_uwb_local_follow/rolling_map_diagnostics
    /go2_uwb_local_follow/planned_cmd
    /go2_uwb_local_follow/final_cmd
    /go2_uwb_local_follow/planner_diagnostics
    /go2_uwb_local_follow/selected_path
    /cmd_vel
    /cmd_vel_planned
    /cmd_vel_follow
    /tf
    /tf_static
    /rosout
    "${MARKER_TOPIC}"
  )
  if [[ "${PROFILE}" == "vision" ]]; then
    topics+=(
      /camera/camera/infra1/image_rect_raw
      /camera/camera/infra1/camera_info
      /camera/camera/infra2/image_rect_raw
      /camera/camera/infra2/camera_info
      /stereo/disparity
      /stereo/depth_debug
    )
  fi
  printf '%s\n' "${topics[@]}" >"${SESSION_DIR}/recorded_topics.txt"

  {
    git -C "${WORKSPACE_ROOT}" status --short --branch
    git -C "${WORKSPACE_ROOT}" log -1 --decorate --oneline
  } >"${SESSION_DIR}/git_state.txt" 2>&1 || true
  git -C "${WORKSPACE_ROOT}" diff >"${SESSION_DIR}/working_tree.diff" 2>&1 || true
  mkdir -p "${SESSION_DIR}/config_snapshot"
  cp -a "${WORKSPACE_ROOT}/src/go2_uwb_local_follow/config/." \
    "${SESSION_DIR}/config_snapshot/" 2>/dev/null || true
  capture_snapshot "start"

  local -a bag_command=(
    ros2 bag record
    --output "${BAG_DIR}"
    --max-bag-size "${MAX_BAG_SIZE_BYTES}"
    --max-cache-size 104857600
    --polling-interval 100
  )
  if [[ "${COMPRESSION}" == "file" ]]; then
    bag_command+=(
      --compression-mode file
      --compression-format zstd
      --compression-threads 1
      --compression-queue-size 2
    )
  fi
  bag_command+=("${topics[@]}")

  "${bag_command[@]}" >"${SESSION_DIR}/rosbag_record.log" 2>&1 &
  BAG_PID=$!
  trap cleanup EXIT
  trap 'exit 130' INT TERM
  sleep 2
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "错误：rosbag 启动失败" >&2
    tail -n 80 "${SESSION_DIR}/rosbag_record.log" >&2 || true
    exit 1
  fi

  monitor_resources &
  MONITOR_PID=$!
  record_event "记录开始" "profile=${PROFILE}"
  echo "输出目录：${SESSION_DIR}"

  if ((DURATION_SEC > 0)); then
    local elapsed=0
    while ((elapsed < DURATION_SEC)) && kill -0 "${BAG_PID}" 2>/dev/null; do
      sleep 1
      elapsed=$((elapsed + 1))
    done
  elif [[ "${NON_INTERACTIVE}" == false && -t 0 ]]; then
    interactive_loop
  else
    echo "非交互记录中，按 Ctrl+C 结束。"
    while kill -0 "${BAG_PID}" 2>/dev/null; do
      sleep 1
    done
  fi
}

main "$@"
