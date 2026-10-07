"""独占离线物理基准：对照请求角速度换向、继续前进时停止转向及全零停车。"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time
import mujoco
import numpy as np
from follow_demo.simulation import Simulation, ROOT
from follow_demo.tests.check_execution import first_sustained, EXPECTED_MODEL_SHA256
from follow_demo.yaw_rate import YawRateServo


CASES = {
    'moving-reversal': (.5, .3),
    'in-place-reversal': (0., .4),
    'moving-rate-limit': (.5, .4),
    'medium-rate-limit': (.3, .4),
    # 能力边界只作诊断，不要求不可达转速通过，不把输入饱和伪装成跟踪成功。
    'in-place-capability': (0., .8),
    # 本轮用户要求的命令边界，独立测量模型能力，不把允许请求等同于准确跟踪。
    'requested-in-place-capability': (0., 1.),
    'requested-moving-capability': (.8, 1.),
}


def sha256(path):
    """记录当前执行源文件和锁定模型，避免混用不同实现的基准。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command_at(stamp, forward, turn, all_zero_at=24.):
    """站稳后正反换向；可在负转末直接全零，单独验证弯中停车。"""
    if stamp < 4. or stamp >= all_zero_at:
        return np.zeros(3)
    angular = turn if stamp < 12. else (-turn if stamp < 20. else 0.)
    return np.array([forward, 0., angular])


def phase_metrics(rows, begin, end, requested):
    """报告原始测量误差和侧移，稳态取阶段末两秒，不删除瞬态失败样本。"""
    phase = [row for row in rows if begin <= row['t'] < end]
    late = [row for row in phase if row['t'] >= end-2.]
    sustained = first_sustained(phase, lambda r: abs(r['averaged_wz']-requested) <= max(.05, abs(requested)*.2))
    direction = first_sustained(phase, lambda r: r['averaged_wz']*np.sign(requested) >= max(.02, abs(requested)*.2)) if requested else None
    origin = phase[0]
    # 以阶段初始机身坐标衡量偏移；原地转向的横移不能用平均vy接近零来掩盖。
    lateral_offsets = [-(r['x']-origin['x'])*math.sin(origin['yaw'])+
                       (r['y']-origin['y'])*math.cos(origin['yaw']) for r in phase]
    return dict(requested_wz=requested, raw_rate_rmse=float(np.sqrt(np.mean([(r['wz']-requested)**2 for r in phase]))),
                late_mean_wz=float(np.mean([r['wz'] for r in late])), late_mean_vx=float(np.mean([r['vx'] for r in late])),
                late_mean_vy=float(np.mean([r['vy'] for r in late])),
                settled_response_s=None if sustained is None else phase[sustained]['t']-begin,
                direction_response_s=None if direction is None else phase[direction]['t']-begin,
                model_input_saturated_ratio=sum(abs(r['execution']['policy_wz']) >= .999 for r in phase)/max(1,len(phase)),
                peak_abs_vy=float(max(abs(r['vy']) for r in phase)),
                max_initial_body_lateral_displacement_m=max(abs(offset) for offset in lateral_offsets),
                phase_displacement_m=math.dist([origin['x'],origin['y']],[phase[-1]['x'],phase[-1]['y']]))


def measure(mode, case, forward, turn, all_zero_at=24.):
    """只用原ONNX与电机PD驱动物理；不启动线程、HTTP或ROS。"""
    sim = Simulation(render=False, execution_mode=mode)
    rows, started = [], time.monotonic()
    dt = sim.model.opt.timestep
    sample_steps = max(1, round(.02/dt))
    try:
        for step in range(round(30./dt)):
            command = command_at(sim.data.time, forward, turn, all_zero_at)
            sim.policy.update(sim.data, command)
            sim.policy.torque(sim.data)
            mujoco.mj_step(sim.model, sim.data)
            if step % sample_steps:
                continue
            sim.update_snapshot(command, False)
            state = sim.snapshot
            row = {key: state[key] for key in ('t','x','y','yaw','vx','vy','wz','z','ready','execution')}
            row['command'] = command.tolist()
            # 带宽判据用显式0.2秒均值抑制步态波动，原始50Hz测量完整保留。
            window = [r['wz'] for r in rows if r['t'] >= row['t']-.2]+[row['wz']]
            row['averaged_wz'] = float(np.mean(window))
            rows.append(row)
        phases = dict(positive=phase_metrics(rows,4.,12.,turn), negative=phase_metrics(rows,12.,20.,-turn),
                      zero_turn=phase_metrics(rows,20.,24.,0.))
        stop_at = 20. if forward == 0. else all_zero_at
        stopped = [r for r in rows if r['t'] >= stop_at]
        stop_index = first_sustained(stopped, lambda r: abs(r['vx']) < .03 and abs(r['vy']) < .03 and abs(r['wz']) < .05)
        last = stopped[stop_index] if stop_index is not None else stopped[-1]
        stop_delta = math.dist([stopped[0]['x'],stopped[0]['y']],[last['x'],last['y']])
        summary = dict(case=case, execution_mode=mode, requested_forward=forward, requested_turn=turn, phases=phases,
                       all_zero_command_at_s=all_zero_at, stop_checks_begin_s=stop_at,
                       stop_time_s=None if stop_index is None else last['t']-stop_at, stop_displacement_m=stop_delta,
                       stop_final_motion={k:rows[-1][k] for k in ('vx','vy','wz')},
                       posture_ready=all(r['ready'] for r in rows if r['t'] >= 4.),
                       wall_elapsed_s=time.monotonic()-started, physics_timestep_s=float(dt))
        # 转向跟踪和姿态/停车分开；超能力诊断不用于给实际模式贴上全通过标签。
        summary['nominal_tracking_met'] = (not case.endswith('capability') and
            all(abs(phases[name]['late_mean_wz']-requested) <= max(.05,abs(requested)*.2)
                for name,requested in (('positive',turn),('negative',-turn),('zero_turn',0.))))
        return summary, rows
    finally:
        sim.close()


def save_results(destination, prefix, report, samples):
    """归档同名旧证据并写独立报告，避免覆盖原执行基准。"""
    destination.mkdir(parents=True,exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for suffix in ('.json','-samples.json'):
        previous = destination/(prefix+suffix)
        if previous.exists():
            history = destination/'history'
            history.mkdir(exist_ok=True)
            shutil.copy2(previous,history/(prefix+'-'+stamp+suffix))
    (destination/(prefix+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (destination/(prefix+'-samples.json')).write_text(json.dumps(samples,ensure_ascii=False),encoding='utf-8')


def main():
    """先获得物理独占锁，记录模型/参数/进程，任何异常都关闭实验资源。"""
    parser = argparse.ArgumentParser(description='独立角速度跟踪对照，不启动ROS/HTTP')
    parser.add_argument('--prefix',default='execution-rate-20261004')
    parser.add_argument('--modes',nargs='+',choices=('heading','rate'),default=['heading','rate'])
    parser.add_argument('--cases',nargs='+',choices=tuple(CASES),
                        default=['moving-reversal','in-place-reversal','in-place-capability'])
    parser.add_argument('--all-zero-at',type=float,choices=(20.,24.),default=24.,
                        help='20秒用于负转中直接停车；24秒用于先停止转向再停车')
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1]
    model = ROOT/'third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx'
    report = dict(kind='physical_yaw_rate_benchmark',schema_version=1,pid=os.getpid(),
                  started_at_utc=datetime.now(timezone.utc).isoformat(),cases=[],
                  rate_configuration=YawRateServo().configuration(),
                  source_sha256={name:sha256(source/name) for name in ('policy.py','yaw_rate.py','simulation.py','tests/check_yaw_rate.py','tests/check_execution.py')},
                  model_sha256=sha256(model),
                  input_profile=dict(stand_until_s=4.,positive_until_s=12.,negative_until_s=20.,
                                     zero_turn_until_s=args.all_zero_at,all_zero_command_at_s=args.all_zero_at,
                                     all_zero_until_s=30.,sample_period_s=.02,
                                     response_rate_average_window_s=.2,sustained_response_window_s=.5),
                  note='有限物理对照，角速度PI和前馈是独立实验；通过不等于导航或实机SDK认证。')
    samples, success = {}, False
    with (ROOT/'follow_demo/runtime.lock').open('a+') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('已有实验台或物理检查运行，未启动角速度对照')
        try:
            if report['model_sha256'] != EXPECTED_MODEL_SHA256:
                raise RuntimeError('模型哈希与锁定模型不同，未执行')
            print(json.dumps(dict(pid=os.getpid(),modes=args.modes,cases=args.cases)),flush=True)
            for mode in args.modes:
                for case in args.cases:
                    summary, rows = measure(mode,case,*CASES[case],all_zero_at=args.all_zero_at)
                    report['cases'].append(summary)
                    samples[mode+'-'+case] = rows
                    print(json.dumps(summary),flush=True)
            success = all(r['posture_ready'] and r['stop_time_s'] is not None for r in report['cases'])
        except Exception as error:
            report['error'] = str(error)
        finally:
            report.update(finished_at_utc=datetime.now(timezone.utc).isoformat(),completed_cases=len(report['cases']),
                          posture_and_stop_checks_passed=success,exit_code=0 if success else 1)
            save_results(ROOT/'artifacts',args.prefix,report,samples)
            print(json.dumps(dict(pid=os.getpid(),exit_code=report['exit_code'],report_path=str(ROOT/'artifacts'/(args.prefix+'.json')))),flush=True)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
