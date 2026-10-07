"""对真实物理闭环做分场景验收，核对测试刺激，保留失败证据并关闭本次服务。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
from urllib.request import Request, build_opener, ProxyHandler
from follow_demo.scenarios import SCENARIOS
from follow_demo.scenario_route import route_points
from follow_demo.scenario_acceptance import loop_completion
from follow_demo.navigation_config import MAX_NAVIGATION_TURN, ROBOT_LENGTH, ROBOT_WIDTH

OPENER = build_opener(ProxyHandler({}))
BASE = 'http://127.0.0.1:8765'
LAST_NAVIGATION = {}
COLLECTION_DIAGNOSTICS = []


def request(path, payload=None):
    """访问本机实验台 API，绕过系统代理，失败直接中止检查。"""
    data = None if payload is None else json.dumps(payload).encode()
    with OPENER.open(Request(BASE + path, data=data, headers={'Content-Type': 'application/json'}), timeout=5) as response:
        return json.load(response)


def action(name, **values):
    """调用与人工验收相同的有限场景操作。"""
    return request('/api/command', {'action': name, **values})


def sample():
    """保存实际运动和刺激身份；完整地图仅保留最后一份，避免每个样本重复写入。"""
    global LAST_NAVIGATION
    state = request('/api/state')
    control = state.get('control', {})
    nav = control.get('navigation', {})
    LAST_NAVIGATION = dict(t=state.get('t'), epoch=state.get('epoch'), scenario=state.get('scenario'),
                           execution_mode=state.get('execution_mode'), execution=state.get('execution'),
                           planning_mode=state.get('planning_mode'),
                           observation_mode=state.get('observation_mode'),
                           camera_pitch_deg=state.get('camera_pitch_deg'),
                           executor_wake_mode=state.get('executor_wake_mode'),
                           search_execution_mode=state.get('search_execution_mode'),
                           depth_fusion=control.get('depth_fusion'),
                           target_mode=state.get('target_mode'), error=state.get('error'), navigation=nav,
                           mppi_error=control.get('mppi_error'))
    if state.get('error') or not state['ros_alive'] or not state['physics_alive']:
        raise RuntimeError(str(state.get('error') or '仿真线程退出'))
    return dict(t=state['t'], control_t=control.get('control_t'),
                control_callback=control.get('control_callback'),sensor_bridge=state.get('sensor_bridge'),
                logging=state.get('logging'), render_diagnostics=state.get('render_diagnostics'),
                scheduling_monitor=state.get('scheduling_monitor'),
                executor_wake_mode=state.get('executor_wake_mode'), executor_pulse=state.get('executor_pulse'),
                search_execution_mode=state.get('search_execution_mode'),
                x=state['x'], y=state['y'], yaw=state['yaw'], ready=state['ready'],
                vx=state['vx'], vy=state.get('vy'), wz=state['wz'], target=state['target'], distance=state['distance'],
                depth_enabled=state.get('depth_enabled'), signal_valid=state.get('signal'),
                epoch=state['epoch'], scenario=state['scenario'], target_mode=state['target_mode'],
                target_route=state.get('target_route'),
                execution_mode=state.get('execution_mode'), execution=state.get('execution'),
                planning_mode=state.get('planning_mode'), follow_intent=nav.get('follow_intent'),
                observation_mode=state.get('observation_mode'),
                camera_pitch_deg=state.get('camera_pitch_deg'),
                depth_fusion=control.get('depth_fusion'),
                contacts=state['obstacle_contacts'], command=control['command'], state=control['state'],
                code=nav.get('code'), depth_age=nav.get('map', {}).get('depth_age'),
                frames=nav.get('map', {}).get('frames'), confirmed=nav.get('map', {}).get('confirmed'),
                path=nav.get('path'), plan_ms=nav.get('plan_ms'), target_velocity=state['target_velocity'],
                estimated_target_velocity=control.get('estimated_target_velocity'),
                speed_reference=nav.get('speed_reference'),
                plan_cpu_ms=nav.get('plan_cpu_ms'),
                search_worker_pids=nav.get('search_worker_pids'),
                search_failed=nav.get('search_failed'),search_error=nav.get('search_error'),
                target_speed=math.hypot(*state['target_velocity']), raw_command=nav.get('raw_command'),
                speed_limit=nav.get('speed_limit'), footprint=nav.get('footprint'),
                heading=nav.get('heading'),facing=nav.get('facing'),command_limits=nav.get('command_limits'),
                execution_guard=nav.get('execution_guard'),
                smoothing=nav.get('smoothing'),
                continuation_limited=nav.get('continuation_limited'),
                odometry_alignment=nav.get('odometry_alignment'),
                halt_counts=nav.get('halt_counts'), halt_events=nav.get('halt_events'),
                look_yaw=nav.get('look_yaw'), plan_revision=nav.get('plan_revision'),
                controller_age=nav.get('controller_age'), observation=nav.get('observation'),
                search_pending=nav.get('search_pending'),
                rejected_plan_count=nav.get('rejected_plan_count'),
                rejected_plan_reason=nav.get('rejected_plan_reason'),
                tracker=nav.get('tracker'), mppi_alive=state.get('mppi_alive'),
                mppi_error=control.get('mppi_error'), mppi_action=control.get('mppi_action'))


def verify_camera_pitch(samples, expected=None):
    """核对真实运行的安装角度，禁止把不同相机覆盖范围的结果混作同一实验。"""
    if not samples:
        raise RuntimeError('没有相机安装角度采样')
    actual = samples[0].get('camera_pitch_deg')
    if not isinstance(actual, (int, float)) or not math.isfinite(actual) or not 0 <= actual <= 40:
        raise RuntimeError('服务未回报有效相机安装俯角')
    if expected is not None and not math.isclose(actual, expected, abs_tol=1e-6):
        raise RuntimeError(f'相机俯角不符：期望 {expected}，实际 {actual}')
    if any(row.get('camera_pitch_deg') != actual for row in samples):
        raise RuntimeError('本轮采样相机俯角发生变化')
    return actual


def verify_execution_mode(samples, expected_mode=None):
    """拒收模式缺失、错配或跨模式采样，避免同一源码的 A/B 结果被错误归类。"""
    if not samples:
        raise RuntimeError('没有执行模式采样，不能核对本次实验身份')
    actual_mode = samples[0].get('execution_mode')
    if actual_mode not in ('heading', 'velocity', 'rate'):
        raise RuntimeError('服务未回报有效执行模式，请更新并重启实验台')
    if expected_mode is not None and actual_mode != expected_mode:
        raise RuntimeError(f'执行模式不符：期望 {expected_mode}，实际 {actual_mode}')
    if any(row.get('execution_mode') != actual_mode for row in samples):
        raise RuntimeError('本次采样的执行模式发生变化，不能作为同一模式的验收证据')
    return actual_mode


def verify_planning_mode(samples, expected_mode=None):
    """核对终点语义身份，旧服务缺失身份或混用规划方式时不能生成 A/B 通过结论。"""
    if not samples:
        raise RuntimeError('没有规划模式采样，不能核对本次实验身份')
    actual_mode = samples[0].get('planning_mode')
    if actual_mode not in ('annulus', 'trail'):
        raise RuntimeError('服务未回报有效规划模式，请更新并重启实验台')
    if expected_mode is not None and actual_mode != expected_mode:
        raise RuntimeError(f'规划模式不符：期望 {expected_mode}，实际 {actual_mode}')
    if any(row.get('planning_mode') != actual_mode for row in samples):
        raise RuntimeError('本次采样的规划模式发生变化，不能作为同一模式的验收证据')
    return actual_mode


def verify_observation_mode(samples, expected_mode=None):
    """禁止将旧扇区与真实标定观察混作同一试验；模型身份与规划身份独立记录。"""
    if not samples:
        raise RuntimeError('没有观察模式采样，不能核对本次实验身份')
    actual_mode = samples[0].get('observation_mode')
    if actual_mode not in ('cone', 'camera'):
        raise RuntimeError('服务未回报有效观察模式，请更新并重启实验台')
    if expected_mode is not None and actual_mode != expected_mode:
        raise RuntimeError(f'观察模式不符：期望 {expected_mode}，实际 {actual_mode}')
    if any(row.get('observation_mode') != actual_mode for row in samples):
        raise RuntimeError('本次采样的观察模式发生变化，不能作为同一模式的验收证据')
    return actual_mode


def verify_executor_wake_mode(samples, expected_mode=None):
    """核对等待修复的 A/B 身份，禁止用未启用修复的缓存服务生成通过结论。"""
    if not samples:
        raise RuntimeError('没有执行器唤醒模式采样')
    actual_mode = samples[0].get('executor_wake_mode')
    if actual_mode not in ('native', 'steady'):
        raise RuntimeError('服务未回报有效执行器唤醒模式，请更新并重启实验台')
    if expected_mode is not None and actual_mode != expected_mode:
        raise RuntimeError(f'执行器唤醒模式不符：期望 {expected_mode}，实际 {actual_mode}')
    if any(row.get('executor_wake_mode') != actual_mode for row in samples):
        raise RuntimeError('本次采样的执行器唤醒模式发生变化')
    return actual_mode


def verify_search_execution_mode(samples, expected_mode=None):
    """区分共享线程与独立进程；同源码不同运行方式不可混作一次性能验收。"""
    if not samples:
        raise RuntimeError('没有搜索运行模式采样')
    actual_mode = samples[0].get('search_execution_mode')
    if actual_mode not in ('thread', 'process'):
        raise RuntimeError('服务未回报有效搜索运行模式，请更新并重启实验台')
    if expected_mode is not None and actual_mode != expected_mode:
        raise RuntimeError(f'搜索运行模式不符：期望 {expected_mode}，实际 {actual_mode}')
    if any(row.get('search_execution_mode') != actual_mode for row in samples):
        raise RuntimeError('本次采样的搜索运行模式发生变化')
    return actual_mode


def collect(duration, result=None):
    """按仿真时间采样，去除同代次的缓存重复；倒退或重置仍保留为失败证据。"""
    result = [] if result is None else result
    start = sample()['t']
    wall_start = time.monotonic()
    deadline = wall_start + duration * 2 + 10
    diagnostic = dict(start_simulation_t=start, requested_duration_s=duration,
                      cached_duplicate_reads=0, longest_duplicate_read_streak=0)
    duplicate_streak = 0
    try:
        while time.monotonic() < deadline:
            row = sample()
            # Telemetry 缓存同一物理快照时，重复读取不是新的时间样本。
            # 只删除同 epoch 的相等时间；时间变小、epoch 变化必须入列表，交给刺激校验判失败。
            if result and row['epoch'] == result[-1]['epoch'] and row['t'] == result[-1]['t']:
                duplicate_streak += 1
                diagnostic['cached_duplicate_reads'] += 1
                diagnostic['longest_duplicate_read_streak'] = max(diagnostic['longest_duplicate_read_streak'], duplicate_streak)
            else:
                result.append(row)
                duplicate_streak = 0
            if row['t'] - start >= duration:
                return result
            time.sleep(0.25)
        raise TimeoutError('仿真时间未正常前进')
    finally:
        diagnostic.update(wall_elapsed_s=time.monotonic() - wall_start, retained_samples=len(result))
        COLLECTION_DIAGNOSTICS.append(diagnostic)


def collect_input_loss(report, key, action_name, duration):
    """保存输入中断的完整时序；额外采样只提供证据，不改变现有停车判定。"""
    record = dict(action=action_name, requested_duration_s=duration, samples=[])
    report.setdefault('input_loss_samples', {})[key] = record
    record['before_action'] = sample()
    record['action_started_wall'] = time.monotonic()
    record['action_reply'] = action(action_name)
    record['action_completed_wall'] = time.monotonic()
    # HTTP 已接收操作不代表 Telemetry 已产生新快照；前后时间相同也按事实记录。
    record['after_action'] = sample()
    diagnostic_index = len(COLLECTION_DIAGNOSTICS)
    try:
        return collect(duration, record['samples'])
    finally:
        # collect 超时时也保留部分样本及重复缓存数量，不能只留下一个失败布尔值。
        if len(COLLECTION_DIAGNOSTICS) > diagnostic_index:
            record['collection'] = dict(COLLECTION_DIAGNOSTICS[-1])


def prepare(scenario, speed=0.8):
    """重载场景、确认起始净空，并返回目标开始行走前的时间与 epoch 基准。"""
    epoch = sample()['epoch']
    action('scenario', scenario=scenario)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        state = sample()
        if state['epoch'] > epoch and state['scenario'] == scenario and state['ready'] and state['frames']:
            break
        time.sleep(0.2)
    else:
        raise TimeoutError('场景重置后未就绪')
    action('confirm_clearance')
    collect(0.5)
    if not sample()['confirmed']:
        raise RuntimeError('起始净空未确认')
    action('resume')
    baseline = sample()
    expected_mode = 'straight' if scenario == 'open' else 'route'
    action(expected_mode, speed=speed)
    # API 状态由异步 Telemetry 更新；先等命令的新快照，避免把命令前的 hold 算进刺激。
    # 时间进度仍使用命令前 baseline，不能通过推迟计时掩盖目标迟动或后续提前停止。
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        echoed = sample()
        if (echoed['epoch'] == baseline['epoch'] and echoed['scenario'] == scenario
                and echoed['target_mode'] == expected_mode and echoed['t'] > baseline['t']):
            return baseline
        time.sleep(.05)
    raise TimeoutError('目标行走命令未在当前场景的新快照中回显')


def route_geometry(points):
    """计算预置折线的累计弧长，供验收独立核对刺激；不读取导航器的规划路径。"""
    cumulative = [0.0]
    for start, end in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(start, end))
    return cumulative


def route_position(points, cumulative, progress):
    """按预期行走路程得到折线位置，终点之后保持不动。"""
    for index, (start, end) in enumerate(zip(points, points[1:])):
        length = cumulative[index + 1] - cumulative[index]
        if progress <= cumulative[index + 1] and length > 0:
            ratio = min(1.0, max(0.0, (progress - cumulative[index]) / length))
            return [start[axis] + ratio * (end[axis] - start[axis]) for axis in (0, 1)]
    return list(points[-1])


def project_to_route(point, points, cumulative):
    """计算到折线的最短距离和投影弧长，防止把另一条路线的完成度算进当前场景。"""
    best = (math.inf, 0.0)
    for index, (start, end) in enumerate(zip(points, points[1:])):
        delta = [end[axis] - start[axis] for axis in (0, 1)]
        square = sum(value * value for value in delta)
        ratio = 0.0 if square == 0 else min(1.0, max(0.0, sum((point[axis] - start[axis]) * delta[axis] for axis in (0, 1)) / square))
        projected = [start[axis] + ratio * delta[axis] for axis in (0, 1)]
        candidate = (math.dist(point, projected), cumulative[index] + ratio * math.sqrt(square))
        if candidate[0] < best[0]:
            best = candidate
    return best


def validate_stimulus(samples, baseline, scenario, speed, duration):
    """逐样本检查真正执行的场景、路线、速度和重置代次，避免刺激错误造成假通过。"""
    specification = SCENARIOS[scenario]
    start = specification.get('start', [2.0, 0.0])
    elapsed = max(0.0, samples[-1]['t'] - baseline['t'])
    # 开阔场景使用无限直行刺激；其 route 字段只是演示点，不能冒充本次终点。
    points = [start, [start[0] + speed * max(duration + 1, elapsed + 1), start[1]]] if scenario == 'open' else route_points(specification)
    looping = bool(specification.get('loop'))
    cumulative = route_geometry(points)
    mode = 'straight' if scenario == 'open' else 'route'
    rows, violations = [], []
    for row in samples:
        route_error, progress = project_to_route(row['target'], points, cumulative)
        timed_progress = speed * max(0.0, row['t'] - baseline['t'])
        expected_progress = timed_progress if looping else min(cumulative[-1], timed_progress)
        expected = route_position(points, cumulative, expected_progress % cumulative[-1] if looping else expected_progress)
        endpoint_error = math.dist(row['target'], points[-1])
        flags = dict(scenario=row['scenario'] == scenario, epoch=row['epoch'] == baseline['epoch'],
                     route=route_error <= .10, timed_progress=math.dist(row['target'], expected) <= .15,
                     speed=math.isfinite(row['target_speed']) and row['target_speed'] <= speed + .02,
                     mode=row['target_mode'] == mode or (not looping and scenario != 'open' and row['target_mode'] == 'hold' and endpoint_error <= .10))
        if looping:
            actual = row.get('target_route') or {}
            flags['loop_progress'] = (actual.get('loop') is True
                                     and abs(actual.get('lap_length_m', 0) - cumulative[-1]) < 1e-6
                                     and abs(actual.get('progress_m', -1) - expected_progress) <= .15
                                     and actual.get('laps') == int(actual.get('progress_m', 0) / cumulative[-1]))
        rows.append(dict(t=row['t'], route_distance_m=route_error, route_progress_m=progress, loop_progress_valid=flags.get('loop_progress', True),
                         expected_progress_m=expected_progress, expected_position_error_m=math.dist(row['target'], expected)))
        # 每个样本都保留路线距离；报告仅列前 20 个异常，完整身份和位置仍在样本文件中。
        row.update(stimulus_route_distance_m=route_error, stimulus_route_progress_m=progress,
                   stimulus_expected_position_error_m=rows[-1]['expected_position_error_m'])
        if not all(flags.values()) and len(violations) < 20:
            violations.append(dict(t=row['t'], failures=[key for key, valid in flags.items() if not valid],
                                   scenario=row['scenario'], epoch=row['epoch'], target_mode=row['target_mode'], target=row['target']))
    checks = dict(start_matches_scenario=math.dist(baseline['target'], start) <= .05,
                  scenario_unchanged=all(row['scenario'] == scenario for row in samples),
                  epoch_unchanged=all(row['epoch'] == baseline['epoch'] for row in samples),
                  monotonic_sample_time=all(b['t'] > a['t'] for a, b in zip(samples, samples[1:])),
                  requested_duration_observed=samples[-1]['t'] - samples[0]['t'] >= duration - .05,
                  target_on_preset_route=all(row['route_distance_m'] <= .10 for row in rows),
                  target_progress_matches_requested_speed=all(row['expected_position_error_m'] <= .15 for row in rows),
                  target_speed_within_request=all(math.isfinite(row['target_speed']) and row['target_speed'] <= speed + .02 for row in samples),
                  target_mode_valid=all(row['target_mode'] == mode or (not looping and scenario != 'open' and row['target_mode'] == 'hold' and math.dist(row['target'], points[-1]) <= .10) for row in samples),
                  loop_progress_valid=all(row['loop_progress_valid'] for row in rows))
    final_expected = route_position(points, cumulative, (speed * elapsed) % cumulative[-1] if looping else min(cumulative[-1], speed * elapsed))
    return dict(valid=all(checks.values()), checks=checks, expected_epoch=baseline['epoch'],
                criteria=dict(maximum_route_error_m=.10, maximum_timed_position_error_m=.15,
                              maximum_speed_error_mps=.02,
                              reason='路线误差允许转折点厘米级截断，时间进度仍须符合设定速度；场景和epoch必须完全一致'),
                mode=mode, preset_polyline=points, expected_final_target=final_expected,
                looping=looping, preset_endpoint=None if scenario == 'open' or looping else points[-1],
                nominal_route_duration_s=None if scenario == 'open' else cumulative[-1] / speed,
                route_distance_max_m=max(row['route_distance_m'] for row in rows),
                expected_position_error_max_m=max(row['expected_position_error_m'] for row in rows),
                observed_scenarios=sorted(set(row['scenario'] for row in samples)),
                observed_epochs=sorted(set(row['epoch'] for row in samples)),
                observed_target_modes=sorted(set(row['target_mode'] for row in samples)), violations=violations)


def motion_metrics(samples):
    """按仿真时间加权运动指标，把短暂停顿与长时间停止都留在报告中。"""
    intervals = [(a, b['t'] - a['t']) for a, b in zip(samples, samples[1:]) if b['t'] > a['t']]
    elapsed = sum(dt for _, dt in intervals)
    steady = [(row, dt) for row, dt in intervals if row['t'] - samples[0]['t'] > 8 and row['target_speed'] > .15]
    moving = [(row, dt) for row, dt in intervals if row['t'] - samples[0]['t'] > 2 and row['target_speed'] > .15]
    slow_seconds = sum(dt for row, dt in moving if abs(row['vx']) < .05)
    longest_slow, current_slow = 0.0, 0.0
    for row, dt in intervals:
        if row['t'] - samples[0]['t'] > 2 and row['target_speed'] > .15 and abs(row['vx']) < .05:
            current_slow += dt
            longest_slow = max(longest_slow, current_slow)
        else:
            current_slow = 0.0
    return dict(mean_actual_vx=sum(row['vx'] * dt for row, dt in intervals) / max(elapsed, .001),
                waiting_seconds=sum(dt for row, dt in intervals if row['state'] == 'WAITING'),
                steady_actual_vx=sum(row['vx'] * dt for row, dt in steady) / max(sum(dt for _, dt in steady), .001),
                steady_observed_seconds=sum(dt for _, dt in steady),
                steady_max_distance=max((row['distance'] for row, _ in steady), default=0),
                target_moving_seconds=sum(dt for _, dt in moving), target_moving_robot_slow_seconds=slow_seconds,
                target_moving_robot_slow_ratio=slow_seconds / max(sum(dt for _, dt in moving), .001),
                target_moving_mean_actual_vx=sum(row['vx'] * dt for row, dt in moving) / max(sum(dt for _, dt in moving), .001),
                target_moving_max_distance=max((row['distance'] for row, _ in moving), default=0),
                target_moving_longest_slow_seconds=longest_slow,
                state_seconds={state: sum(dt for row, dt in intervals if row['state'] == state) for state in sorted(set(row['state'] for row in samples))},
                code_seconds={code: sum(dt for row, dt in intervals if row['code'] == code) for code in sorted(set(row['code'] for row in samples), key=str)},
                max_actual_wz=max(abs(row['wz']) for row in samples),
                max_command_delta_v_per_s=max((abs(b['command'][0] - a['command'][0]) / (b['t'] - a['t']) for a, b in zip(samples, samples[1:]) if b['t'] > a['t']), default=0))


def angle_metrics(samples):
    """分别统计移动路径跟踪与停人看向，不将绕障时人方位差当作控制精度。"""
    moving = [row['heading'] for row in samples if row.get('heading') and row['state'] in ('FOLLOWING','DETOUR')
              and row['vx']>.25 and row['heading'].get('path_error') is not None]
    errors = [abs(item['path_error'])*180/math.pi for item in moving]
    return dict(moving_samples=len(errors),path_error_rms_deg=(math.sqrt(sum(v*v for v in errors)/len(errors)) if errors else None),
                path_error_max_deg=max(errors,default=None),
                facing_samples=sum(row['state']=='FACING' for row in samples),
                meaning='行走对齐路径；人方位另列，单点UWB不测量人体自身朝向')


def task_completion(samples, scenario, report):
    """要求目标完成正确刺激且机器人真正通过障碍；最后两秒的状态必须稳定。"""
    tail = [row for row in samples if samples[-1]['t'] - row['t'] <= 2.0]
    tail_complete = len(tail) >= 2 and tail[-1]['t'] - tail[0]['t'] >= 1.5
    if SCENARIOS[scenario].get('loop'):
        return loop_completion(samples, route_points(SCENARIOS[scenario]), report['required_laps'], report)
    def x_extent(row):
        """矩形在世界x轴的投影半长随实际朝向变化，验收不再假定圆形半径。"""
        return ROBOT_LENGTH/2*abs(math.cos(row['yaw']))+ROBOT_WIDTH/2*abs(math.sin(row['yaw']))
    if scenario == 'blocked':
        # 封闭场景只需在墙前稳定停住，不要求机器人通过无法通行的墙体。
        wall_near = min(x - sx for x, _, sx, _, _ in SCENARIOS[scenario]['boxes'])
        checks = dict(final_window_observed=tail_complete,
                      target_reached_preset_endpoint=math.dist(samples[-1]['target'], SCENARIOS[scenario]['route'][-1]) <= .10,
                      stopped_before_wall=all(row['state'] == 'WAITING' and row['command'] == [0.0, 0.0] and row['x'] + x_extent(row) < wall_near for row in tail))
        return dict(met=all(checks.values()), checks=checks, far_side_required=False,
                    final_target=samples[-1]['target'], preset_endpoint=SCENARIOS[scenario]['route'][-1])
    checks = dict(physical_progress=report['checks']['physical_progress'], final_window_observed=tail_complete)
    if scenario == 'open':
        checks.update(open_forward_progress=samples[-1]['x'] > 3.9)
        return dict(met=all(checks.values()), checks=checks, far_side_required=False, final_target=samples[-1]['target'])
    # corner 的第二面墙沿通道延伸；它定义边界，横向阻挡只取第一面箱体。
    barrier_indices = {'long_wall': [0], 'consecutive': [0, 1], 'corner': [0]}[scenario]
    far_edge = max(SCENARIOS[scenario]['boxes'][index][0] + SCENARIOS[scenario]['boxes'][index][2] for index in barrier_indices)
    far_side = far_edge + ROBOT_LENGTH/2
    endpoint = SCENARIOS[scenario]['route'][-1]
    checks.update(target_reached_preset_endpoint=all(math.dist(row['target'], endpoint) <= .10 for row in tail),
                  robot_fully_on_far_side=all(row['x']-x_extent(row) >= far_edge for row in tail),
                  final_follow_distance_reasonable=all(.85 <= row['distance'] <= 2.5 for row in tail))
    return dict(met=all(checks.values()), checks=checks, far_side_required=True, far_side_robot_center_x_m=far_side,
                criteria=dict(robot_footprint=dict(shape='rectangle',length_m=ROBOT_LENGTH,width_m=ROBOT_WIDTH),
                              footprint_check='actual_yaw_projection',final_window_s=2.0,
                              maximum_endpoint_error_m=.10, final_distance_range_m=[.85, 2.5],
                              reason='整机跨过横向障碍后，目标须到预置终点并稳定保持约2.1m的正常跟随间距'),
                transverse_barrier_indices=barrier_indices, final_target=samples[-1]['target'], preset_endpoint=endpoint,
                final_target_endpoint_error_m=math.dist(samples[-1]['target'], endpoint),
                final_window_distance_range_m=[min(row['distance'] for row in tail), max(row['distance'] for row in tail)])


def fluency_checks(scenario, speed, report):
    """独立验收跟随速度和停顿；基础安全通过不能代替连续跟随的产品要求。"""
    if scenario == 'blocked':
        return dict(applicable=False, passed=True, checks={}, reason='封闭通道要求稳定停车，不评价持续跟随速度')
    checks = dict(moving_period_observed=report['target_moving_seconds'] >= 3,
                  moving_follow_distance_bounded=report['target_moving_max_distance'] < 3.5,
                  slow_time_ratio_bounded=report['target_moving_robot_slow_ratio'] <= .15,
                  longest_slow_spell_bounded=report['target_moving_longest_slow_seconds'] <= 2.0)
    if scenario == 'open':
        checks.update(steady_window_observed=report['steady_observed_seconds'] >= 5,
                      steady_actual_speed=report['steady_actual_vx'] >= max(.45, speed * .9),
                      steady_follow_distance_bounded=report['steady_max_distance'] < 3.5)
    else:
        checks.update(moving_actual_speed=report['target_moving_mean_actual_vx'] >= speed * .7)
    return dict(applicable=True, passed=all(checks.values()), checks=checks,
                criteria=dict(open_nominal_speed_mps=speed, open_allowed_speed_error_fraction=.1,
                              open_absolute_minimum_steady_speed_mps=.45,
                              lower_speed_runs_are_diagnostic=True,
                              detour_moving_mean_speed_fraction=.7, maximum_moving_follow_distance_m=3.5,
                              maximum_slow_time_ratio=.15, maximum_single_slow_spell_s=2.0),
                reason='开阔处检验设定速度的稳态跟随；绕障允许减速和短停，持续掉队或长停单独判为不流畅')


def debug_reasons(report):
    """列出尚未达标的具体门槛及最终地图/控制线索，避免只给出一个失败布尔值。"""
    reasons = []
    groups = [('基础安全', report.get('basic_safety', {})), ('测试刺激', report.get('stimulus', {})),
              ('任务完成', report.get('task_completion', {})), ('流畅性', report.get('fluency', {}))]
    for name, group in groups:
        for key, valid in group.get('checks', {}).items():
            if not valid:
                reasons.append(f'{name}未通过：{key}')
    if report.get('error'):
        reasons.append('检查中断：' + report['error'])
    if not report.get('passed_navigation'):
        diagnostic = report.get('final_navigation', {})
        nav = diagnostic.get('navigation', {})
        reasons.append(f"最后导航状态：{nav.get('code')}；路径点数={len(nav.get('path') or [])}；地图模式={nav.get('map', {}).get('mode')}；深度年龄={nav.get('map', {}).get('depth_age')}")
        if diagnostic.get('mppi_error'):
            reasons.append('MPPI 返回：' + str(diagnostic['mppi_error']))
    return reasons


def save_report(destination, prefix, scenario, report, samples):
    """写入新运行证据前归档同名旧结果，避免新验收规则覆盖旧报告充当新证据。"""
    destination.mkdir(parents=True, exist_ok=True)
    suffix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for ending in ('.json', '-samples.json'):
        path = destination / f'{prefix}-{scenario}{ending}'
        if path.exists():
            history = destination / 'history'
            history.mkdir(exist_ok=True)
            shutil.copy2(path, history / f'{prefix}-{scenario}-{suffix}{ending}')
    (destination / f'{prefix}-{scenario}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    (destination / f'{prefix}-{scenario}-samples.json').write_text(json.dumps(samples, ensure_ascii=False), encoding='utf-8')


def main():
    """分别验收安全、刺激、完成度和流畅性；任何一项失败都保留证据并返回失败。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=list(SCENARIOS), default='open')
    parser.add_argument('--duration', type=float, default=None,
                        help='省略时使用30秒；开阔直行按目标速度提前避开x=18 m活动边界')
    parser.add_argument('--laps', type=int, default=2, help='循环场景人和狗都需要完成的最少圈数')
    parser.add_argument('--speed', type=float, default=0.8)
    parser.add_argument('--prefix', default='navigation-v2')
    parser.add_argument('--execution-mode', choices=('heading', 'velocity', 'rate'), default=None,
                        help='核对服务实际执行模式；省略时仍记录并检查本次模式一致性')
    parser.add_argument('--planning-mode', choices=('annulus', 'trail'), default=None,
                        help='核对局部跟随终点语义；目标历史不能替代地图通行性')
    parser.add_argument('--observation-mode', choices=('cone', 'camera'), default=None,
                        help='核对实际相机地面支持或旧二维扇区的观察任务身份')
    parser.add_argument('--executor-wake-mode', choices=('native', 'steady'), default=None,
                        help='核对底层原生等待或独立单调唤醒身份')
    parser.add_argument('--search-execution-mode', choices=('thread', 'process'), default=None,
                        help='核对共享 Python 线程或独立 spawn 搜索进程的身份')
    parser.add_argument('--camera-pitch-deg', type=float, default=None,
                        help='核对仿真实际安装俯角；不能当作实机外参')
    arguments = parser.parse_args()
    if not 1 <= arguments.laps <= 20:
        parser.error('循环验收圈数必须为1至20')
    if not math.isfinite(arguments.speed) or not .05 <= arguments.speed <= .80:
        parser.error('目标速度必须在 0.05–0.80 m/s 范围内')
    if arguments.duration is None:
        # 开阔场景从x=2 m直行，达到活动边界会被截停；不能把该截停当成导航掉队。
        arguments.duration = min(30, math.floor(16 / arguments.speed)-1) if arguments.scenario == 'open' else 30
    if not math.isfinite(arguments.duration) or arguments.duration <= 0:
        parser.error('时长必须为正')
    source = Path(__file__).resolve().parents[1]
    hashes = {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in
              ('controller.py', 'local_map.py', 'local_planner.py', 'navigation.py', 'simulation.py', 'scenarios.py', 'ros_nodes.py',
               'policy.py', 'forward_speed.py', 'speed_reference.py', 'native_braking.py', 'command_smoothing.py', 'mppi_bridge.py', 'mppi_runtime.py', 'observation.py', 'observation_geometry.py', 'navigation_config.py',
               'app.py', 'ui_render.py', 'setup.bash', 'cyclonedds.xml', 'camera_profile.py', 'yaw_rate.py', 'follow_intent.py', 'executor_pulse.py',
               'footprint.py','heading_guide.py','static/index.html', 'tests/check_navigation_demo.py',
               'scenario_route.py', 'scenario_acceptance.py')}
    # 控制器新增原生评分器，Python源码哈希不足以标识实际运行的控制器版本。
    native = Path.home() / 'go2_sim/follow_native'
    plugin_identity = {str(path.relative_to(native)): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in (native / 'source.sha256', native / 'lib/libgo2_follow_mppi_critics.so')}
    specification = SCENARIOS[arguments.scenario]
    # 嵌入本轮几何并保存摘要，未来场景文件扩展也不改变已有报告的障碍解释。
    specification_hash = hashlib.sha256(json.dumps(specification, ensure_ascii=False, sort_keys=True).encode()).hexdigest()
    report, samples = {'acceptance_schema_version': 7, 'scenario': arguments.scenario, 'checks': {}, 'source_sha256': hashes,
                       'native_plugin_sha256': plugin_identity,
                       'execution_assumptions': dict(world='flat_static',
                           projected_depth_age_is_actuation_delay=False,
                           command_hold_seconds=.25,linear_brake_deceleration_mps2=.45,
                           positive_following_terminal_exit_m=.6,
                           terminal_exit_required_for_stop=False,
                           angular_brake_deceleration_radps2=1.,unknown_is_blocked=True,
                           note='仿真名义执行参数，不能替代实机延迟和制动标定'),
                       'scenario_specification': specification, 'scenario_spec_sha256': specification_hash,
                       'required_laps': arguments.laps,
                       'started_at_utc': datetime.now(timezone.utc).isoformat(), 'requested_duration_s': arguments.duration,
                       'target_speed_mps': arguments.speed, 'passed_basic_checks': False, 'stimulus_valid': False,
                       'expected_execution_mode': arguments.execution_mode, 'execution_mode': None,
                       'execution_mode_verified': False,
                       'expected_planning_mode': arguments.planning_mode, 'planning_mode': None,
                       'planning_mode_verified': False,
                       'expected_observation_mode': arguments.observation_mode, 'observation_mode': None,
                       'observation_mode_verified': False,
                       'expected_camera_pitch_deg': arguments.camera_pitch_deg, 'camera_pitch_deg': None,
                       'camera_pitch_verified': False,
                       'expected_executor_wake_mode': arguments.executor_wake_mode, 'executor_wake_mode': None,
                       'executor_wake_mode_verified': False,
                       'expected_search_execution_mode': arguments.search_execution_mode, 'search_execution_mode': None,
                       'search_execution_mode_verified': False,
                       'validation_mode': 'nominal_following_acceptance' if arguments.speed >= .5 else 'lower_speed_diagnostic',
                       'scenario_objective_met': False, 'passed_fluency': False, 'passed_navigation': False}, []
    try:
        # 同一源码可以启动不同执行模式，源码哈希不足以区分 A/B，必须核对实际服务。
        initial = sample()
        actual_mode = initial.get('execution_mode')
        report['execution_mode'] = actual_mode
        verify_execution_mode([initial], arguments.execution_mode)
        report['planning_mode'] = verify_planning_mode([initial], arguments.planning_mode)
        report['observation_mode'] = verify_observation_mode([initial], arguments.observation_mode)
        report['camera_pitch_deg'] = verify_camera_pitch([initial], arguments.camera_pitch_deg)
        report['executor_wake_mode'] = verify_executor_wake_mode([initial], arguments.executor_wake_mode)
        report['search_execution_mode'] = verify_search_execution_mode([initial], arguments.search_execution_mode)
        baseline = prepare(arguments.scenario, arguments.speed)
        report['stimulus_start'] = {key: baseline[key] for key in ('t', 'epoch', 'scenario', 'target_mode', 'target')}
        collect(arguments.duration, samples)
        verify_execution_mode([initial, baseline, *samples], arguments.execution_mode)
        verify_planning_mode([initial, baseline, *samples], arguments.planning_mode)
        verify_observation_mode([initial, baseline, *samples], arguments.observation_mode)
        verify_camera_pitch([initial, baseline, *samples], arguments.camera_pitch_deg)
        verify_executor_wake_mode([initial, baseline, *samples], arguments.executor_wake_mode)
        verify_search_execution_mode([initial, baseline, *samples], arguments.search_execution_mode)
        report['execution_mode_verified'] = True
        report['planning_mode_verified'] = True
        report['observation_mode_verified'] = True
        report['camera_pitch_verified'] = True
        report['executor_wake_mode_verified'] = True
        report['search_execution_mode_verified'] = True
        report['final_navigation'] = LAST_NAVIGATION
        displacement = math.dist([samples[0]['x'], samples[0]['y']], [samples[-1]['x'], samples[-1]['y']])
        traveled = sum(math.dist([a['x'], a['y']], [b['x'], b['y']]) for a, b in zip(samples, samples[1:]))
        report.update(displacement_m=displacement, final_distance_m=samples[-1]['distance'],
                      final_position=[samples[-1]['x'], samples[-1]['y']], final_target=samples[-1]['target'], final_code=samples[-1]['code'],
                      states=sorted(set(row['state'] for row in samples)), plan_ms_max=max(row['plan_ms'] or 0 for row in samples))
        report['checks'].update(no_physical_obstacle_contact=all(row['contacts'] == 0 for row in samples),
                                posture_ready=all(row['ready'] for row in samples),
                                phase_speed_limits=all(0 <= row['command'][0] <= (row.get('speed_limit') or 0) + 1e-6
                                                       and abs(row['command'][1]) <= MAX_NAVIGATION_TURN+1e-6 for row in samples),
                                nav2_mppi_alive=all(row.get('mppi_alive') and row.get('tracker') == 'nav2_mppi' for row in samples),
                                # 闭环回到起点时净位移可以为零；实际通行还须通过有序路段门校验。
                                physical_progress=traveled > 4.0 if specification.get('loop') else displacement > .4)
        report['actual_traveled_distance_m'] = traveled
        report.update(motion_metrics(samples))
        report['angle_metrics'] = angle_metrics(samples)
        report['stimulus'] = validate_stimulus(samples, baseline, arguments.scenario, arguments.speed, arguments.duration)
        report['stimulus_valid'] = report['stimulus']['valid']
        report['task_completion'] = task_completion(samples, arguments.scenario, report)
        report['scenario_objective_met'] = report['task_completion']['met']
        report['reached_far_side'] = report['task_completion']['checks'].get('robot_fully_on_far_side', False)
        report['fluency'] = fluency_checks(arguments.scenario, arguments.speed, report)
        report['passed_fluency'] = report['fluency']['passed']
        stopped = collect_input_loss(report, 'depth', 'depth_off', 2.5)
        report['checks']['depth_loss_stops'] = len(stopped) >= 4 and all(row['command'] == [0.0, 0.0] and row['code'] == 'MAP_STALE' for row in stopped[-4:])
        report['depth_loss_final_motion'] = {key: stopped[-1][key] for key in ('vx', 'wz')}
        action('depth_on')
        collect(1.5)
        stopped = collect_input_loss(report, 'uwb', 'signal_off', 1.0)
        report['checks']['uwb_loss_stops'] = len(stopped) >= 2 and all(row['command'] == [0.0, 0.0] and row['state'] == 'TARGET_LOST' for row in stopped[-2:])
        # 中断检查验证命令及时归零；实际速度另存，不能声称零命令就是零制动距离。
        report['uwb_loss_final_motion'] = {key: stopped[-1][key] for key in ('vx', 'wz')}
        safety_keys = ('no_physical_obstacle_contact', 'posture_ready', 'phase_speed_limits', 'depth_loss_stops', 'uwb_loss_stops', 'nav2_mppi_alive')
        report['basic_safety'] = dict(checks={key: report['checks'][key] for key in safety_keys})
        report['passed_basic_checks'] = all(report['basic_safety']['checks'].values())
        report['basic_safety']['passed'] = report['passed_basic_checks']
        report['passed_safety_and_progress'] = all(report['checks'].values())
        report['passed_navigation'] = (report['passed_basic_checks'] and report['stimulus_valid']
                                       and report['scenario_objective_met'] and report['passed_fluency']
                                       and report['execution_mode_verified'] and report['planning_mode_verified']
                                       and report['observation_mode_verified'] and report['executor_wake_mode_verified']
                                       and report['search_execution_mode_verified'] and report['camera_pitch_verified'])
    except Exception as error:
        report['error'] = str(error)
        report['passed_safety_and_progress'], report['passed_navigation'] = False, False
    finally:
        report.setdefault('final_navigation', LAST_NAVIGATION)
        report['collection_diagnostics'] = COLLECTION_DIAGNOSTICS
        report['debug_reasons'] = debug_reasons(report)
        try:
            action('estop')
            request('/api/shutdown', {})
        except Exception as error:
            report['shutdown_request_error'] = str(error)
        # 服务先停止再写文件，存储错误也不能把本次仿真留在后台。
        destination = Path.home() / 'go2_sim/artifacts'
        save_report(destination, arguments.prefix, arguments.scenario, report, samples)
        # 终端只输出决策摘要；完整路径与地图 cells 保存在 JSON，避免一万多字符栅格淹没失败原因。
        summary = {key: report.get(key) for key in ('scenario', 'validation_mode', 'execution_mode', 'planning_mode', 'observation_mode', 'camera_pitch_deg',
                   'expected_execution_mode', 'execution_mode_verified', 'passed_basic_checks', 'stimulus_valid',
                   'scenario_objective_met', 'passed_fluency', 'passed_navigation', 'steady_actual_vx',
                   'target_moving_mean_actual_vx', 'final_distance_m', 'target_moving_robot_slow_ratio', 'debug_reasons')}
        summary['report_path'] = str(destination / f'{arguments.prefix}-{arguments.scenario}.json')
        summary['samples_path'] = str(destination / f'{arguments.prefix}-{arguments.scenario}-samples.json')
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if report.get('passed_navigation') else 1


if __name__ == '__main__':
    raise SystemExit(main())
