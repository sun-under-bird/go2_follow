"""只读核对速度修复的完整场景、原生二进制、配置身份及本次进程退出。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
from urllib.request import build_opener, ProxyHandler


SCENARIOS=('open','long_wall','consecutive','corner','blocked','square_loop','slalom_loop','wall_loop')


def digest(path):
    """用实际文件字节核对版本，缺文件不能冒充相同源码。"""
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def read(path):
    """读取Windows及WSL的UTF-8 JSON，不改动报告或采样。"""
    return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    """审计结果与导航通过标志分开保存；失败场景同样可以是有效的实验记录。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--prefix',required=True)
    arguments=parser.parse_args()
    root=Path(__file__).resolve().parent
    source=root.parent/'follow_demo'
    linux=Path(r'\\wsl.localhost\Ubuntu-22.04\home\chy\go2_sim')
    scenes=[]
    owned_pids=set()
    for scenario in SCENARIOS:
        report=read(root/f'{arguments.prefix}-{scenario}.json')
        rows=read(root/f'{arguments.prefix}-{scenario}-samples.json')
        hashes=report['source_sha256']
        mismatches=[name for name,value in hashes.items() if digest(source/name)!=value]
        binary_mismatches=[name for name,value in report['native_plugin_sha256'].items()
                           if digest(linux/'follow_native'/name)!=value]
        for row in rows:
            owned_pids.update(row.get('search_worker_pids') or [])
            worker=(row.get('render_diagnostics') or {}).get('ui_process_pid')
            if worker:
                owned_pids.add(worker)
        checks=dict(source_matches=not mismatches,native_matches=not binary_mismatches,
                    pitch_verified=report.get('camera_pitch_verified') is True,
                    pitch_consistent=bool(rows) and all(row.get('camera_pitch_deg')==25. for row in rows),
                    modes_consistent=bool(rows) and all(tuple(row.get(key) for key in
                        ('execution_mode','planning_mode','observation_mode','executor_wake_mode','search_execution_mode'))
                        ==('rate','trail','camera','steady','process') for row in rows),
                    sample_identity=bool(rows) and all(row.get('scenario')==scenario for row in rows),
                    time_increasing=bool(rows) and all(b['t']>a['t'] for a,b in zip(rows,rows[1:])),
                    epoch_consistent=len({row['epoch'] for row in rows})==1,
                    collected=not report.get('error'),stimulus=report.get('stimulus_valid') is True)
        scenes.append(dict(scenario=scenario,checks=checks,evidence_valid=all(checks.values()),
                           passed=report['passed_navigation'],basic_safety=report['passed_basic_checks'],
                           task=report['scenario_objective_met'],fluency=report['passed_fluency'],
                           mean_vx=report['target_moving_mean_actual_vx'],steady_vx=report['steady_actual_vx'],
                           slow_ratio=report['target_moving_robot_slow_ratio'],
                           max_gap=report.get('target_moving_max_distance'),
                           longest_slow=report.get('target_moving_longest_slow_seconds'),
                           robot_laps=report.get('task_completion',{}).get('robot',{}).get('completed_laps'),
                           reasons=report.get('debug_reasons'),source_sha256=hashes,
                           native_plugin_sha256=report['native_plugin_sha256'],
                           report_sha256=digest(root/f'{arguments.prefix}-{scenario}.json'),
                           samples_sha256=digest(root/f'{arguments.prefix}-{scenario}-samples.json')))
    # 只读取进程列表，并只保留本实验台命令；不终止任何进程或保存其他任务的命令。
    process=subprocess.run(['wsl.exe','-d','Ubuntu-22.04','-u','chy','--exec','ps','-eo','pid,ppid,args'],
                           text=True,capture_output=True,check=True)
    remaining=[]
    for line in process.stdout.splitlines()[1:]:
        fields=line.strip().split(None,2)
        if len(fields)!=3:
            continue
        pid,_,command=fields
        task_command=(command.startswith('python -m follow_demo.app ') or
                      '/go2_follow_mppi' in command and command.startswith('/opt/ros/humble/lib/nav2_') or
                      int(pid) in owned_pids and 'multiprocessing.spawn' in command)
        if task_command:
            remaining.append(dict(pid=int(pid),command=command))
    http_alive=False
    try:
        with build_opener(ProxyHandler({})).open('http://127.0.0.1:8765/api/state',timeout=2) as response:
            http_alive=response.status==200
    except OSError:
        pass
    same_source=all(scene['source_sha256']==scenes[0]['source_sha256'] for scene in scenes)
    same_native=all(scene['native_plugin_sha256']==scenes[0]['native_plugin_sha256'] for scene in scenes)
    # 使用安装器相同的路径和排序规则，补核对本机C++源码与已编译库的来源标记。
    # 标记包含原始绝对路径，仅证明本机版本；异机复现仍需要重新构建。
    command=('find "/mnt/c/Users/chy/Documents/ChatGPT/go2_slam 2/sim_env/scripts/../native/go2_follow_mppi_critics" '
             '-type f -print0 | sort -z | xargs -0 sha256sum | sha256sum')
    native_source=subprocess.run(['wsl.exe','-d','Ubuntu-22.04','-u','chy','--exec','bash','-lc',command],
                                 text=True,capture_output=True,check=True)
    native_source_digest=native_source.stdout.split()[0]
    native_source_matches=native_source_digest==(linux/'follow_native/source.sha256').read_text().strip()
    result=dict(prefix=arguments.prefix,checked_at_utc=datetime.now(timezone.utc).isoformat(),
                same_source=same_source,same_native=same_native,
                native_source_digest=native_source_digest,native_source_matches=native_source_matches,
                source_file_count=len(scenes[0]['source_sha256']),
                evidence_valid=same_source and same_native and native_source_matches and all(scene['evidence_valid'] for scene in scenes),
                passed_scenes=sum(scene['passed'] for scene in scenes),scenarios=scenes,
                cleanup=dict(http_alive=http_alive,remaining_task_processes=remaining,
                             sampled_worker_pids=sorted(owned_pids),verified=not http_alive and not remaining))
    destination=root/f'{arguments.prefix}-final-audit.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({key:result[key] for key in ('evidence_valid','passed_scenes','same_source','same_native','native_source_matches','cleanup')},ensure_ascii=False))
    return 0 if result['evidence_valid'] and result['cleanup']['verified'] else 1


if __name__=='__main__':
    raise SystemExit(main())
