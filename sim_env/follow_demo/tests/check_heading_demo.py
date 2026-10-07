"""通过真实仿真验证左右停人看向，记录命令、实际角度和足端接触。"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import time
import re
from .check_navigation_demo import action,request,collect


def main():
    """依次重置两个侧向目标；本模块不关闭服务，由独占的PowerShell入口回收。"""
    parser = argparse.ArgumentParser(description='独立验证左右停人看向，使用独立证据前缀')
    parser.add_argument('--prefix',default='heading-rectangle')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,63}',args.prefix):
        parser.error('报告前缀仅允许小写字母、数字和短横线，最长64字符')
    root = Path(__file__).resolve().parents[1]
    names = ('navigation.py','navigation_config.py','heading_guide.py','footprint.py','local_planner.py',
             'observation.py','observation_geometry.py','simulation.py','mppi_runtime.py','mppi_bridge.py',
             'yaw_rate.py','tests/check_heading_demo.py')
    report = dict(cases=[],passed=False,started_at_utc=datetime.now(timezone.utc).isoformat(),
                  source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names})
    output = Path.home()/f'go2_sim/artifacts/{args.prefix}.json'
    try:
        for label,angle in (('left',.6),('right',-.6)):
            action('scenario',scenario='open')
            deadline = time.monotonic()+30
            while not request('/api/state')['ready']:
                if time.monotonic()>deadline:
                    raise TimeoutError('姿态未就绪')
                time.sleep(.2)
            action('pause')
            action('confirm_clearance')
            action('waypoint',x=1.8*math.cos(angle),y=1.8*math.sin(angle),speed=.5)
            collect(5.)
            action('resume')
            rows = collect(12.)
            tail = [row for row in rows if rows[-1]['t']-row['t']<=2]
            errors = [abs(math.atan2(math.sin(math.atan2(row['target'][1]-row['y'],row['target'][0]-row['x'])-row['yaw']),
                                     math.cos(math.atan2(row['target'][1]-row['y'],row['target'][0]-row['x'])-row['yaw']))) for row in tail]
            checks = dict(facing_executed=any(row['state']=='FACING' for row in rows),
                          no_forward_command=all(abs(row['command'][0])<1e-9 for row in rows),
                          no_contact=all(row['contacts']==0 for row in rows),
                          final_angle_within_seven_degrees=max(errors)<math.radians(7),
                          final_holding=all(row['state']=='HOLDING' for row in tail))
            case = dict(name=label,goal_angle_rad=angle,checks=checks,passed=all(checks.values()),
                        final_error_deg=max(errors)*180/math.pi,samples=rows)
            report['cases'].append(case)
            print(label,checks,case['final_error_deg'],flush=True)
        report['passed'] = all(case['passed'] for case in report['cases'])
    except Exception as error:
        report['error'] = str(error)
    finally:
        try:
            action('pause')
        finally:
            report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
