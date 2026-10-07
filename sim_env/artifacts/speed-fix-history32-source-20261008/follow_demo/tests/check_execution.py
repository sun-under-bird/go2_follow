"""离线推进真实 MuJoCo/ONNX，测量执行响应并记录上游契约；不发布 ROS 命令。"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
import mujoco
from follow_demo.simulation import Simulation, ROOT


CASES = {
    'rate-straight-0.4': ('rate', .4, 0.),
    'rate-straight-0.6': ('rate', .6, 0.),
    'rate-straight-0.8': ('rate', .8, 0.),
    'rate-turn-left-0.8': ('rate', .8, .3),
    'rate-turn-right-0.8': ('rate', .8, -.3),
    # 紧弯能力诊断与普通速度验收分开运行；请求上限不能当作模型可达转速。
    'rate-tight-turn-left-0.8': ('rate', .8, .8),
    'rate-tight-turn-right-0.8': ('rate', .8, -.8),
    'rate-tight-turn-left-0.4': ('rate', .4, .8),
    'velocity-straight-0.12': ('velocity', .12, 0.),
    'velocity-straight-0.2': ('velocity', .2, 0.),
    'velocity-straight-0.3': ('velocity', .3, 0.),
    'velocity-straight-0.5': ('velocity', .5, 0.),
    'velocity-straight-0.7': ('velocity', .7, 0.),
    'velocity-slow-turn': ('velocity', .12, .15),
    'heading-slow-turn': ('heading', .12, .15),
    'velocity-moving-turn': ('velocity', .5, .3),
    'heading-moving-turn': ('heading', .5, .3),
    'velocity-in-place': ('velocity', 0., .4),
    'heading-in-place': ('heading', 0., .4),
}
EXPECTED_MODEL_SHA256 = '1b3261d32f1c7b157a98a4f66ac689c89ec10a5032ad9a9a67bd96d4556d6901'
REVIEWED_UPSTREAM_SHA256 = 'c048d57c691d3cb913d3b45b172035471e22c47348a3b609ec1744572630869a'


def sha256(path):
    """记录实际读取文件的哈希，确保执行对照没有悄悄换模型或适配层。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def first_sustained(rows, predicate, window=.5):
    """寻找连续达到速度条件至少半秒的起点，避免把步态瞬时过零当成启动或停稳。"""
    for index, row in enumerate(rows):
        if not predicate(row):
            continue
        end = index
        while end + 1 < len(rows) and rows[end]['t'] - row['t'] < window:
            end += 1
            if not predicate(rows[end]):
                break
        if rows[end]['t'] - row['t'] >= window and all(predicate(value) for value in rows[index:end + 1]):
            return index
    return None


def measure(case, mode, forward, turn):
    """站稳4秒、执行恒定指令16秒、停车6秒；只用原策略关节力矩改变物理状态。"""
    sim = Simulation(render=False)
    sim.policy.yaw_mode = mode
    rows, started = [], time.monotonic()
    dt = sim.model.opt.timestep
    sample_steps = max(1, round(.02 / dt))
    try:
        for step in range(round(26.0 / dt)):
            t = sim.data.time
            command = np.array([forward, 0.0, turn]) if 4 <= t < 20 else np.zeros(3)
            sim.policy.update(sim.data, command)
            sim.policy.torque(sim.data)
            mujoco.mj_step(sim.model, sim.data)
            if step % sample_steps == 0:
                sim.update_snapshot(command, False)
                state = sim.snapshot
                row = {key: state[key] for key in ('t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'z', 'ready')}
                row.update(command=command.tolist(), heading_target=float(sim.policy.heading),
                           forward_servo=dict(sim.policy.forward_servo.last))
                rows.append(row)
        moving = [row for row in rows if 16 <= row['t'] < 20]
        early = [row for row in rows if 8 <= row['t'] < 12]
        active = [row for row in rows if 4 <= row['t'] < 20]
        stopped = [row for row in rows if row['t'] >= 20]
        stop_index = first_sustained(stopped, lambda r: abs(r['vx']) < .03 and abs(r['vy']) < .03 and abs(r['wz']) < .05)
        stop_row = stopped[stop_index] if stop_index is not None else stopped[-1]
        startup_index = first_sustained(active, lambda r: abs(r['vx']) >= max(.02, abs(forward) * .2)) if forward else None
        turning_index = first_sustained(active, lambda r: abs(r['wz']) >= max(.02, abs(turn) * .2)) if turn else None
        braking_rows = stopped[:stop_index + 1] if stop_index is not None else stopped
        yaw = np.unwrap([row['yaw'] for row in active])
        summary = dict(case=case, yaw_mode=mode, command=[forward, turn],
                       mean_model_forward_input=float(np.mean([r['forward_servo']['model_input'] for r in moving])),
                       mean_vx=float(np.mean([r['vx'] for r in moving])),
                       mean_vy=float(np.mean([r['vy'] for r in moving])),
                       mean_wz=float(np.mean([r['wz'] for r in moving])),
                       std_vx=float(np.std([r['vx'] for r in moving])),
                       std_wz=float(np.std([r['wz'] for r in moving])),
                       early_mean_vx=float(np.mean([r['vx'] for r in early])),
                       early_mean_wz=float(np.mean([r['wz'] for r in early])),
                       moving_yaw_change_rad=float(yaw[-1] - yaw[0]),
                       translation_response_s=None if startup_index is None else active[startup_index]['t'] - 4,
                       turning_response_s=None if turning_index is None else active[turning_index]['t'] - 4,
                       stop_time_s=None if stop_index is None else stop_row['t'] - 20,
                       stop_displacement_m=float(np.hypot(stop_row['x'] - stopped[0]['x'], stop_row['y'] - stopped[0]['y'])),
                       stop_path_length_m=float(sum(np.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(braking_rows, braking_rows[1:]))),
                       stop_final_motion={key: stopped[-1][key] for key in ('vx', 'vy', 'wz')},
                       posture_ready=all(r['ready'] for r in rows if r['t'] > 4),
                       policy_inference_count=sim.policy.inference_count, physics_timestep_s=dt,
                       wall_elapsed_s=time.monotonic() - started,
                       outside_upstream_documented_linear_range=abs(forward) > .5,
                       forward_response_ratio=None if not forward else float(np.mean([r['vx'] for r in moving])) / forward,
                       angular_response_ratio=None if not turn else float(np.mean([r['wz'] for r in moving])) / turn)
        # 中高速rate校准案例必须实测跟上，而不能只凭姿态正常、停车成功声称速度已修好。
        applicable = case.startswith('rate-') and forward >= .4
        summary['speed_tracking'] = dict(applicable=applicable, linear_tolerance_mps=.04,
                                        angular_tolerance_radps=.08,
                                        passed=abs(summary['mean_vx']-forward)<=.04 and
                                               abs(summary['mean_wz']-turn)<=.08 if applicable else None)
        return summary, rows
    finally:
        # render=False且未start，不创建服务或线程；仍走close以保持资源清理契约。
        sim.close()


def interface_audit():
    """保存逐项源码审阅结果；heading对照只切现有适配模式，不冒充完整上游C++程序。"""
    return dict(
        basis='锁定部署C++源码与MuJoCo XML；原始NP3O训练配置未随此部署仓库提供，训练范围仅引用部署源码说明',
        no_observation_layout_bug_found=True,
        observation_layout=[dict(indices='0:3', meaning='机体系IMU陀螺仪', scale=.25),
                            dict(indices='3:6', meaning='机体系投影重力，wxyz四元数', scale=1.),
                            dict(indices='6:9', meaning='vx、vy、模型角速度命令', scale=[2., 2., .25]),
                            dict(indices='9:21', meaning='SDK顺序关节角减默认姿态', scale=1.),
                            dict(indices='21:33', meaning='SDK顺序关节速度', scale=.05),
                            dict(indices='33:45', meaning='上次未滤波策略动作', scale=1.)],
        joint_order='FR、FL、RR、RL；每腿hip、thigh、calf', input_shapes=dict(obs=[1, 45], hist=[1, 10, 45]),
        history='首拍推理接收零历史；推理之后首帧填满10拍历史，随后移位后加入当前观测，两实现相同',
        action_filter=dict(new_weight=.8, previous_filtered_weight=.2, action_scale=.25, hip_reduction=.5),
        pd=dict(kp=40., kd=1., target_velocity=0., feedforward_torque=0.), policy_period_s=.02,
        upstream_source_lines=dict(layout='15-28、304-336', history='339-350、500-504', yaw='455-478',
                                   constant_yaw_warning='456-460', action_filter_pd='483-498'),
        real_differences=[
            '上游所有模式先维护航向目标，再由clip(0.5*wrap(目标航向-实测yaw),±1)生成模型wz；当前velocity模式直接给模型wz。',
            '上游源码记录恒定wz曾转约0.56rad后停止；这是上游经验说明，本次独立对照才用于验证当前仿真响应。',
            '上游有3秒线速度淡入；本基准所有模式都用相同阶跃，不增加起步或死区补偿。',
            '当前heading模式在全零速度时重置航向目标，上游在普通零指令时保留旧航向；停车结果不能冒充完整上游停车行为。',
            '当前物理步长来自MuJoCo模型，推理50Hz；部署C++仅通过DDS发送PD参数，当前每物理步直接重算同一PD力矩。'],
        conclusion='未发现布局、历史或动作PD错接；转向命令生成契约确有差异，模式优劣由本次有限时长物理对照评估。')


def save_results(destination, prefix, report, samples):
    """独立保存本轮基准，重跑时归档同名前次证据，保留旧execution-v2结果。"""
    destination.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for suffix in ('.json', '-samples.json'):
        path = destination / (prefix + suffix)
        if path.exists():
            history = destination / 'history'
            history.mkdir(exist_ok=True)
            shutil.copy2(path, history / (prefix + '-' + stamp + suffix))
    (destination / (prefix + '.json')).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    (destination / (prefix + '-samples.json')).write_text(json.dumps(samples, ensure_ascii=False), encoding='utf-8')


def main():
    """独占实验台物理资源，记录PID/源码/模型及接口审阅；有限用例结束后退出。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', default='execution-region-20261004')
    parser.add_argument('--cases', nargs='+', choices=list(CASES), default=list(CASES))
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1]
    upstream = ROOT / 'third_party/capo-sim-assets'
    model = upstream / 'sim_patch/models/go2_policy.onnx'
    runner = upstream / 'sim_patch/example_cpp/policy_runner.cpp'
    report = dict(kind='physical_execution_benchmark', benchmark_schema_version=3, pid=os.getpid(),
                  command_line=sys.argv, started_at_utc=datetime.now(timezone.utc).isoformat(), cases=[],
                  input_profile=dict(case_duration_s=26., stand_until_s=4., motion_until_s=20.,
                                     steady_window_s=[16., 20.], sample_interval_s=.02,
                                     stop_velocity_limits=dict(vx=.03, vy=.03, wz=.05), stop_hold_s=.5),
                  interface_audit=interface_audit(),
                  source_sha256={name: sha256(source / name) for name in ('policy.py', 'forward_speed.py', 'simulation.py', 'camera_profile.py', 'scenarios.py', 'tests/check_execution.py')},
                  asset_sha256={str(path.relative_to(ROOT)): sha256(path) for path in
                                (model, runner, upstream / 'sim_patch/scripts/export_onnx.py',
                                 ROOT / 'third_party/unitree_mujoco/unitree_robots/go2/go2.xml',
                                 ROOT / 'third_party/unitree_mujoco/unitree_robots/go2/scene.xml')},
                  note='有限时长开环模型输入基准；姿态/停车通过不等于速度跟踪、导航绕障或实机接口认证。')
    samples, success = {}, False
    ROOT.joinpath('follow_demo').mkdir(parents=True, exist_ok=True)
    with ROOT.joinpath('follow_demo/runtime.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('请先关闭跟随实验台，执行基准不能与其同时运行。')
        try:
            if sha256(model) != EXPECTED_MODEL_SHA256 or sha256(runner) != REVIEWED_UPSTREAM_SHA256:
                raise RuntimeError('模型或上游接口源码与本次已审阅锁定版本不一致，未开始物理基准')
            report['upstream_commit'] = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
            print(json.dumps(dict(pid=report['pid'], started_at_utc=report['started_at_utc'], cases=args.cases)), flush=True)
            for case in args.cases:
                mode, forward, turn = CASES[case]
                summary, rows = measure(case, mode, forward, turn)
                report['cases'].append(summary)
                samples[case] = rows
                print(json.dumps(summary), flush=True)
            success = all(row['posture_ready'] and row['stop_time_s'] is not None and
                          row['speed_tracking']['passed'] is not False for row in report['cases'])
        except Exception as error:
            report['error'] = str(error)
        finally:
            report.update(finished_at_utc=datetime.now(timezone.utc).isoformat(), completed_cases=len(report['cases']),
                          planned_cases=len(args.cases), all_cases_completed=len(report['cases']) == len(args.cases),
                          posture_and_stop_checks_passed=bool(report['cases']) and all(
                              row['posture_ready'] and row['stop_time_s'] is not None for row in report['cases']),
                          all_selected_checks_passed=success, exit_code=0 if success else 1)
            save_results(ROOT / 'artifacts', args.prefix, report, samples)
            print(json.dumps(dict(pid=report['pid'], exit_code=report['exit_code'], report_path=str(ROOT / 'artifacts' / (args.prefix + '.json')))), flush=True)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
