"""验证交互接口的连续调速、地图目标、日志下载与输入边界。"""
import csv
import io
import json
import math
from pathlib import Path
import time
from urllib.error import HTTPError
from check_live_demo import action, request, OPENER, BASE


def main():
    """执行有限时长的交互检查；结束时重置为暂停状态。"""
    checks = {}
    try:
        action('reset')
        time.sleep(0.3)
        action('circle', speed=0.15)
        time.sleep(2)
        before = request('/api/state')
        action('speed', speed=0.45)
        time.sleep(0.2)
        after = request('/api/state')
        # 用实际仿真时间约束移动距离，速度调整不能让目标瞬移。
        displacement = math.dist(before['target'], after['target'])
        checks['speed_change_is_continuous'] = after['target_mode'] == 'circle' and displacement <= 0.45 * (after['t'] - before['t']) + 0.01
        goal = [after['target'][0], after['target'][1] + 1]
        action('waypoint', x=goal[0], y=goal[1], speed=0.3)
        time.sleep(1)
        waypoint = request('/api/state')
        checks['waypoint_moves_target'] = math.dist(waypoint['target'], goal) < 0.95
        with OPENER.open(BASE + '/api/log.csv', timeout=5) as response:
            data = response.read()
            rows = list(csv.reader(io.StringIO(data.decode())))
            checks['csv_snapshot_is_complete'] = len(data) == int(response.headers['Content-Length']) and len(rows) > 2 and all(len(row) == len(rows[0]) for row in rows)
        try:
            request('/api/command', {'action': 'waypoint', 'x': 100, 'y': 0})
            checks['invalid_waypoint_rejected'] = False
        except HTTPError as error:
            checks['invalid_waypoint_rejected'] = error.code == 422
    finally:
        action('reset')
    report = dict(passed=bool(checks) and all(checks.values()), checks=checks)
    (Path.home() / 'go2_sim/artifacts/follow-demo-ui-api-check.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
