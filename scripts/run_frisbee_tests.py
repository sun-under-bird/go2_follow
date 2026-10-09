#!/usr/bin/env python3
"""在隔离 ROS 域运行本仓库测试，并回收本次测试进程组."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def group_processes(group_id):
    """只枚举指定测试进程组，避免影响用户的其他 ROS、WSL 或 Docker 任务."""
    rows = subprocess.check_output(["ps", "-eo", "pid=,pgid=,args="], text=True)
    return [line.strip() for line in rows.splitlines()
            if len(line.split(None, 2)) >= 2 and int(line.split(None, 2)[1]) == group_id]


def run_owned(command, environment, timeout_sec):
    """独立启动测试进程组；正常、失败、超时和 Ctrl+C 都进入定向清理."""
    process = subprocess.Popen(command, env=environment, start_new_session=True)
    print(f"本次测试进程组 PGID={process.pid}", flush=True)
    try:
        return process.wait(timeout=timeout_sec)
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if not group_processes(process.pid):
                break
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            time.sleep(0.2)
        remaining = group_processes(process.pid)
        if remaining:
            raise RuntimeError(f"本次测试进程尚未退出: {remaining}")
        print(f"测试进程组 {process.pid} 已全部停止", flush=True)


def main():
    """选择数据阶段或全量回归，允许使用独立的 WSL 构建目录."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("data", "approach", "all"), default="all")
    parser.add_argument("--build-base", default="build")
    parser.add_argument("--timeout-sec", type=float, default=600.0)
    parser.add_argument("--ctest-regex", help="只复跑指定测试名正则，仍执行进程清理和结果检查")
    args = parser.parse_args()
    os.chdir(Path(__file__).resolve().parents[1])
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = environment.get("FRISBEE_TEST_DOMAIN_ID", "177")
    environment["ROS_LOCALHOST_ONLY"] = "1"
    # 本机 WSL 的 FastDDS 可发现节点但不能交付样本；仅测试时选用现有 CycloneDDS。
    if "RMW_IMPLEMENTATION" not in environment and Path(
            "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so").exists():
        environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    if (environment.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp" and
            "CYCLONEDDS_URI" not in environment):
        # 仅调整测试进程的 DDS 传输；生产节点和已有用户配置不受影响。
        # Humble 的 CycloneDDS 不解码文件 URI 中的空格，直接传入 XML 内容。
        environment["CYCLONEDDS_URI"] = Path(__file__).with_name(
            "cyclonedds_test.xml").read_text(encoding="utf-8")
    command = ["colcon", "test", "--build-base", args.build_base,
               "--executor", "sequential", "--packages-select",
               "uwb_aoa_pkg", "go2_uwb_local_follow"]
    if args.phase == "approach":
        command = command[:-2]
    if args.phase in ("all", "approach"):
        command.append("go2_uwb_behavior")
    command += ["--event-handlers", "console_direct+"]
    if args.ctest_regex:
        command += ["--ctest-args", "-R", args.ctest_regex]
    elif args.phase == "data":
        command += ["--ctest-args", "-R",
                    "test_(uart_stack|serial_reconnect|uwb_target_store|multi_uwb_launch)"]
    elif args.phase == "approach":
        command += ["--ctest-args", "-R", "test_(approach|retrieve_client)"]
    result = run_owned(command, environment, args.timeout_sec)
    # colcon test 的执行退出码不代表断言全通过，必须检查实际测试报告。
    report = subprocess.call(["colcon", "test-result", "--test-result-base", args.build_base,
                              "--verbose"], env=environment)
    return result or report


if __name__ == "__main__":
    raise SystemExit(main())
