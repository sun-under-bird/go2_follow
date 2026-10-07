"""启动发行版自带的 Nav2 MPPI；只管理本实验台拥有的子进程。"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from .navigation_config import (FOLLOW_GOAL_TOLERANCE, OBSERVATION_POSITION_TOLERANCE,
                                OBSERVATION_YAW_TOLERANCE, MAX_NAVIGATION_TURN, MAX_NAVIGATION_SPEED,
                                ROBOT_LENGTH, ROBOT_WIDTH, MPPI_HORIZON_SECONDS,
                                PATH_ANGLE_SOFT_THRESHOLD,FACE_EXIT_ANGLE)

ROOT = Path.home()/'go2_sim'
MANIFEST = ROOT/'follow_demo/runtime/mppi-processes.json'


def parameters():
    """生成 Humble 参数；两种跟踪器共享模型，仅观察终点需要完成指定朝向。"""
    common = dict(plugin='nav2_mppi_controller::MPPIController', time_steps=round(MPPI_HORIZON_SECONDS/.05), model_dt=.05,
                  batch_size=800, iteration_count=1, vx_std=.25, vy_std=0., wz_std=.20,
                  vx_max=MAX_NAVIGATION_SPEED, vx_min=0., vy_max=0., wz_max=MAX_NAVIGATION_TURN, motion_model='DiffDrive',
                  prune_distance=3.5, transform_tolerance=.2, temperature=.3, gamma=.003,
                  visualize=False, retry_attempt_limit=1, reset_period=1.,
                  critics=['ConstraintCritic','CostCritic','BrakingCritic','FollowSpeedCritic','GoalCritic','PathFollowCritic','PathAlignCritic','PathAngleCritic','GoalAngleCritic'],
                  BrakingCritic=dict(enabled=True,cost_weight=1000.,robot_length=ROBOT_LENGTH,robot_width=ROBOT_WIDTH,
                                     continuation_distance=.6),
                  FollowSpeedCritic=dict(enabled=True,cost_weight=18.,reference_timeout=.3,
                                        reference_decay_seconds=.6,maximum_speed=MAX_NAVIGATION_SPEED),
                  ConstraintCritic=dict(enabled=True,cost_power=1,cost_weight=4.),
                  CostCritic=dict(enabled=True,cost_power=1,cost_weight=3.81,critical_cost=300.,
                                  consider_footprint=True,collision_cost=1000000.,near_goal_distance=.3),
                  GoalCritic=dict(enabled=True,cost_power=1,cost_weight=5.,threshold_to_consider=1.4),
                  PathFollowCritic=dict(enabled=True,cost_power=1,cost_weight=5.,offset_from_furthest=5,threshold_to_consider=.15),
                  PathAlignCritic=dict(enabled=True,cost_power=1,cost_weight=6.,offset_from_furthest=8,
                                       threshold_to_consider=.3,trajectory_point_step=3,max_path_occupancy_ratio=.05),
                  PathAngleCritic=dict(enabled=True,cost_power=1,cost_weight=5.,offset_from_furthest=4,
                                       threshold_to_consider=FOLLOW_GOAL_TOLERANCE,max_angle_to_furthest=PATH_ANGLE_SOFT_THRESHOLD,
                                       forward_preference=True),
                  GoalAngleCritic=dict(enabled=False,cost_power=1,cost_weight=5.,threshold_to_consider=.35))
    # ROS 参数解析器不支持 YAML 引用；深拷贝防止序列化器为共享列表/字典生成别名。
    observe = copy.deepcopy(common)
    # 短观察路线在到达前仍需沿途朝向约束；原 0.30 m 关闭范围会让 0.1～0.3 m 的剩余路线失去转向引导。
    # 固定位置的看向任务只有终点朝向，没有下一段行驶方向，因此关闭路径方向代价。
    observe['PathAngleCritic']['enabled'] = False
    observe['GoalAngleCritic'] = dict(enabled=True,cost_power=1,cost_weight=8.,threshold_to_consider=.35)
    controller = dict(use_sim_time=True,controller_frequency=30.,odom_topic='/follow_demo/odom',
                      min_x_velocity_threshold=.001,min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001,
                      failure_tolerance=.3,progress_checker_plugin='progress',goal_checker_plugins=['follow_goal','observe_goal','face_goal'],
                      controller_plugins=['FollowPath','Observe'],
                      progress=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.15,movement_time_allowance=20.),
                      follow_goal=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=FOLLOW_GOAL_TOLERANCE,yaw_goal_tolerance=6.28,stateful=False),
                      observe_goal=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=OBSERVATION_POSITION_TOLERANCE,
                                        yaw_goal_tolerance=OBSERVATION_YAW_TOLERANCE,stateful=False),
                      face_goal=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=OBSERVATION_POSITION_TOLERANCE,
                                     yaw_goal_tolerance=FACE_EXIT_ANGLE,stateful=False),
                      FollowPath=common,Observe=observe)
    costmap = dict(use_sim_time=True,update_frequency=10.,publish_frequency=2.,global_frame='odom',robot_base_frame='base_footprint',
                   rolling_window=True,width=12,height=12,resolution=.1,
                   footprint=str([[ROBOT_LENGTH/2,ROBOT_WIDTH/2],[-ROBOT_LENGTH/2,ROBOT_WIDTH/2],
                                  [-ROBOT_LENGTH/2,-ROBOT_WIDTH/2],[ROBOT_LENGTH/2,-ROBOT_WIDTH/2]]),
                   footprint_padding=0.,
                   track_unknown_space=True,always_send_full_costmap=True,transform_tolerance=.2,
                   plugins=['geometry','inflation'],
                   geometry=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/follow_demo/local_geometry',
                                 map_subscribe_transient_local=True,subscribe_to_updates=False),
                   inflation=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.8,cost_scaling_factor=5.))
    return {'/go2_follow_mppi/controller_server':{'ros__parameters':controller},
            '/go2_follow_mppi/local_costmap/local_costmap':{'ros__parameters':costmap},
            '/go2_follow_mppi/lifecycle_manager':{'ros__parameters':dict(use_sim_time=True,autostart=True,
                 node_names=['controller_server'],bond_timeout=0.)}}


def identity(pid):
    """用进程启动时刻和命令验证 PID，防止 PID 复用后误终止别的任务。"""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode()
        return dict(pid=pid,start_ticks=fields[19],command=command)
    except (FileNotFoundError,ProcessLookupError):
        return None


def matches(record):
    """每次升级退出信号前重新核对完整身份，不能只依赖启动时保存的 PID。"""
    current = identity(record['pid'])
    if current is None:
        return False
    if current != record or '/go2_follow_mppi' not in current['command'] or os.getpgid(record['pid']) != record['pid']:
        raise RuntimeError('Nav2 子进程身份或独立进程组已变化，拒绝终止')
    return True


def stop_owned():
    """只清理清单中仍匹配身份的专属 Nav2 进程组，正常退出和意外退出均可使用。"""
    if not MANIFEST.exists():
        return
    records = json.loads(MANIFEST.read_text())
    for record in records:
        if matches(record):
            os.killpg(record['pid'],signal.SIGINT)
    deadline = time.monotonic()+5
    while time.monotonic() < deadline and any(identity(r['pid']) for r in records):
        time.sleep(.05)
    for record in records:
        if matches(record):
            os.killpg(record['pid'],signal.SIGTERM)
    deadline = time.monotonic()+2
    while time.monotonic() < deadline and any(identity(r['pid']) for r in records):
        time.sleep(.05)
    if any(identity(r['pid']) for r in records):
        raise RuntimeError('专属 Nav2 子进程仍未退出，请查看其日志')
    MANIFEST.unlink(missing_ok=True)


class MppiRuntime:
    """用独立进程运行 C++ 控制器，Python 搜索不会挤占其计算线程。"""
    def __init__(self):
        """保存已创建进程，任何启动失败都能清理此前成功启动的部分。"""
        self.processes,self.logs,self.records = [],[],[]

    def start(self):
        """启动控制器和生命周期管理器；节点职责在启动项旁明确标注。"""
        if MANIFEST.exists():
            existing = json.loads(MANIFEST.read_text())
            if any(identity(item['pid']) for item in existing):
                raise RuntimeError('已有本实验台 Nav2 子进程，请先运行停止脚本')
        config = ROOT/'follow_demo/runtime/mppi.yaml'
        config.write_text(yaml.safe_dump(parameters(),sort_keys=False))
        # 控制器服务器：接收连续路径，输出尚未经过本项目执行保护的 MPPI 候选速度。
        # 生命周期管理器：配置并激活唯一的控制器服务器，不管理其他 ROS 节点。
        for package,node in [('nav2_controller','controller_server'),('nav2_lifecycle_manager','lifecycle_manager')]:
            binary = f'/opt/ros/humble/lib/{package}/{node}'
            log = ROOT/'logs'/f'mppi-{node}.log'
            output = log.open('w')
            self.logs.append(output)
            args = [binary,'--ros-args','-r','__ns:=/go2_follow_mppi','--params-file',str(config)]
            if node == 'controller_server':
                args += ['-r','cmd_vel:=/follow_demo/mppi_cmd']
            process = subprocess.Popen(args,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            self.processes.append(process)
            # Popen返回后子进程可能仍处于fork后的Python映像，等待exec完成再记录身份。
            # 否则清单记下的是父入口命令，停止时会误判为PID已复用。
            deadline, record = time.monotonic()+1., None
            while time.monotonic() < deadline and process.poll() is None:
                candidate = identity(process.pid)
                if candidate and candidate['command'].startswith(binary+' ') and os.getpgid(process.pid) == process.pid:
                    record = candidate
                    break
                time.sleep(.005)
            if record is None:
                raise RuntimeError(f'{node} 未完成启动，请查看 {log}')
            self.records.append(record)
            MANIFEST.write_text(json.dumps(self.records))

    def close(self):
        """请求子进程正常退出并回收；超时仅终止仍由本对象持有的进程。"""
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=2)
        for log in self.logs:
            log.close()
        if MANIFEST.exists():
            MANIFEST.unlink()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--stop-owned',action='store_true')
    if parser.parse_args().stop_owned:
        stop_owned()
