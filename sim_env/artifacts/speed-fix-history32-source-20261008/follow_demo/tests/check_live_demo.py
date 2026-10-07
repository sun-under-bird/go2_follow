"""针对正在运行的演示进行实际跟随、信号中断、暂停与重置检查。"""
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen, build_opener, ProxyHandler

BASE = 'http://127.0.0.1:8765'
OPENER = build_opener(ProxyHandler({}))
SAMPLES = []


def request(path, payload=None):
    """通过本机 HTTP 接口执行与页面相同的操作。"""
    data = None if payload is None else json.dumps(payload).encode()
    message = Request(BASE + path, data=data, headers={'Content-Type': 'application/json'})
    with OPENER.open(message, timeout=5) as response:
        return json.load(response)


def action(name, **values):
    """提交一个已定义的场景操作。"""
    request('/api/command', {'action': name, **values})


def collect(duration):
    """按真实墙钟观察物理运动，不以发出的命令代替执行结果。"""
    deadline = time.monotonic() + duration
    result = []
    while time.monotonic() < deadline:
        state = request('/api/state')
        if state.get('error') or not state.get('ros_alive') or not state.get('physics_alive'):
            raise RuntimeError(str(state.get('error') or '运行线程异常'))
        compact = {key: state[key] for key in ('t', 'x', 'y', 'z', 'yaw', 'vx', 'wz', 'distance', 'ready', 'signal', 'control')}
        result.append(compact)
        SAMPLES.append(compact)
        time.sleep(0.2)
    return result


def main():
    """保存逐阶段结果；检查结束后停目标、暂停跟随，便于用户接手。"""
    report = {'checks': {}, 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    try:
        action('reset')
        collect(5)
        start = request('/api/state')
        action('resume')
        action('straight', speed=0.30)
        walking = collect(30)
        last = walking[-1]
        displacement = math.hypot(last['x'] - start['x'], last['y'] - start['y'])
        report['checks']['physical_follow'] = {'passed': displacement > 0.5 and all(row['ready'] for row in walking),
                                              'displacement_m': displacement, 'final_distance_m': last['distance'],
                                              'minimum_height_m': min(row['z'] for row in walking)}
        action('hold')
        stopping = collect(12)
        report['checks']['target_stop'] = {'passed': stopping[-1]['control']['command'][0] < walking[-1]['control']['command'][0],
                                          'final_command_vx': stopping[-1]['control']['command'][0],
                                          'final_actual_vx': stopping[-1]['vx'], 'final_distance_m': stopping[-1]['distance']}
        action('signal_off')
        lost = collect(2)
        report['checks']['target_loss'] = {'passed': all(row['control']['state'] == 'TARGET_LOST'
                                                        and row['control']['command'] == [0.0, 0.0] for row in lost[3:])}
        action('signal_on')
        action('straight', speed=0.30)
        collect(4)
        action('pause')
        paused = collect(1)
        report['checks']['pause'] = {'passed': paused[-1]['control']['state'] == 'PAUSED' and paused[-1]['control']['command'] == [0.0, 0.0]}
        old = request('/api/state')
        action('reset')
        collect(1)
        reset = request('/api/state')
        report['checks']['reset_clock'] = {'passed': reset['epoch'] == old['epoch'] + 1 and reset['t'] > old['t'] and not reset['enabled']}
    except Exception as error:
        report['error'] = str(error)
    finally:
        action('hold')
        action('pause')
        report['passed'] = 'error' not in report and bool(report['checks']) and all(item['passed'] for item in report['checks'].values())
        destination = Path.home() / 'go2_sim/artifacts'
        (destination / 'follow-demo-check.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        (destination / 'follow-demo-samples.json').write_text(json.dumps(SAMPLES, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
