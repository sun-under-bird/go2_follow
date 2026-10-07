"""检查实际步态策略能否跟随连续转弯的目标，不用指令角速度冒充实际转向。"""
import json
import math
from pathlib import Path
from check_live_demo import action, collect, request, SAMPLES


def main():
    """执行半径三米的目标绕弧行走，完成后重置并暂停。"""
    report = {}
    try:
        action('reset')
        collect(5)
        initial = request('/api/state')
        action('resume')
        action('circle', speed=0.30)
        samples = collect(30)
        final = samples[-1]
        yaw_change = math.atan2(math.sin(final['yaw'] - initial['yaw']), math.cos(final['yaw'] - initial['yaw']))
        report = dict(yaw_change_rad=yaw_change, final_distance_m=final['distance'],
                      maximum_distance_m=max(row['distance'] for row in samples),
                      minimum_height_m=min(row['z'] for row in samples),
                      final_robot_position=[final['x'], final['y']],
                      passed=yaw_change > 1.0 and all(row['ready'] for row in samples)
                             and max(row['distance'] for row in samples) < 3.5)
    except Exception as error:
        report = dict(passed=False, error=str(error))
    finally:
        action('reset')
        destination = Path.home() / 'go2_sim/artifacts'
        (destination / 'follow-demo-turn-check.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        (destination / 'follow-demo-turn-samples.json').write_text(json.dumps(SAMPLES))
        print(json.dumps(report, indent=2, ensure_ascii=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
