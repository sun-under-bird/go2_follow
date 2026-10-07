"""离线核验全向方案所需的横移能力；不启动 ROS、网页或改变当前导航模型。"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import time

import mujoco
import numpy as np

from follow_demo.simulation import Simulation, ROOT
from follow_demo.tests.check_execution import first_sustained, EXPECTED_MODEL_SHA256, save_results


CASES = {'left': (0.,.25,0.), 'right': (0.,-.25,0.),
         'diagonal_left': (.4,.2,0.), 'diagonal_right': (.4,-.2,0.), 'reverse': (-.2,0.,0.)}


def measure(case, command):
    """站稳4秒、移动8秒、停车6秒；记录实际三轴速度及完整停止过程。"""
    simulation = Simulation(render=False,execution_mode='rate')
    rows, started = [],time.monotonic()
    dt = simulation.model.opt.timestep
    try:
        for step in range(round(18./dt)):
            stamp = simulation.data.time
            request = np.asarray(command) if 4. <= stamp < 12. else np.zeros(3)
            simulation.policy.update(simulation.data,request)
            simulation.policy.torque(simulation.data)
            mujoco.mj_step(simulation.model,simulation.data)
            if step % max(1,round(.02/dt)):
                continue
            simulation.update_snapshot(request,False)
            state = simulation.snapshot
            row = {key:state[key] for key in ('t','x','y','yaw','vx','vy','wz','z','ready','execution')}
            row['command'] = request.tolist()
            rows.append(row)
        steady = [row for row in rows if 10. <= row['t'] < 12.]
        stopped = [row for row in rows if row['t'] >= 12.]
        index = first_sustained(stopped,lambda row: math.hypot(row['vx'],row['vy'])<.03 and abs(row['wz'])<.05)
        stop_window = stopped if index is None else stopped[:index+1]
        summary = dict(case=case,requested_command=list(command),
                       steady_actual={key:float(np.mean([row[key] for row in steady])) for key in ('vx','vy','wz')},
                       stop_time_s=None if index is None else stopped[index]['t']-12.,
                       stop_path_length_m=sum(math.dist([a['x'],a['y']],[b['x'],b['y']])
                                              for a,b in zip(stop_window,stop_window[1:])),
                       stop_final_motion={key:stopped[-1][key] for key in ('vx','vy','wz')},
                       posture_ready=all(row['ready'] for row in rows if row['t']>=4.),
                       wall_elapsed_s=time.monotonic()-started)
        # 姿态/停车与速度可跟踪分开；横移均值正常也不能认证瞬态、制动模型或实机 Sport 接口。
        summary['steady_tracking_met'] = all(abs(summary['steady_actual'][key]-value)<=max(.05,abs(value)*.2)
                                              for key,value in zip(('vx','vy','wz'),command))
        return summary,rows
    finally:
        simulation.close()


def main():
    """先拿物理资源独占锁，保存 PID、模型和源码身份，全部结束后退出。"""
    parser = argparse.ArgumentParser(description='横移与倒退有限物理基准，不改变默认导航')
    parser.add_argument('--prefix',default='execution-lateral-20261005')
    parser.add_argument('--cases',nargs='+',choices=list(CASES),default=list(CASES))
    arguments = parser.parse_args()
    source = Path(__file__).resolve().parents[1]
    model = ROOT/'third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx'
    report = dict(kind='physical_lateral_benchmark',pid=os.getpid(),execution_mode='rate',cases=[],
                  started_at_utc=datetime.now(timezone.utc).isoformat(),
                  model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  source_sha256={name:hashlib.sha256((source/name).read_bytes()).hexdigest()
                                 for name in ('policy.py','yaw_rate.py','simulation.py','tests/check_lateral_execution.py')},
                  input_profile=dict(stand_until_s=4.,motion_until_s=12.,stop_until_s=18.,sample_interval_s=.02),
                  note='有限无障碍能力基准；未改导航为Omni，不能代替碰撞校验、前视观察或实机验收。')
    samples = {}
    with (ROOT/'follow_demo/runtime.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            if report['model_sha256'] != EXPECTED_MODEL_SHA256:
                raise RuntimeError('模型哈希不符，未执行')
            print(json.dumps(dict(pid=report['pid'],cases=arguments.cases)),flush=True)
            for case in arguments.cases:
                summary,rows = measure(case,CASES[case])
                report['cases'].append(summary)
                samples[case] = rows
                print(json.dumps(summary,ensure_ascii=False),flush=True)
        except Exception as error:
            report['error'] = str(error)
        finally:
            report['all_cases_completed'] = len(report['cases'])==len(arguments.cases)
            report['posture_and_stop_checks_passed'] = report['all_cases_completed'] and all(
                row['posture_ready'] and row['stop_time_s'] is not None for row in report['cases'])
            report['steady_tracking_met'] = report['all_cases_completed'] and all(
                row['steady_tracking_met'] for row in report['cases'])
            report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            save_results(ROOT/'artifacts',arguments.prefix,report,samples)
    return 0 if report['posture_and_stop_checks_passed'] and report['steady_tracking_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
