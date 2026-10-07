"""用同进程双执行器核对墙钟回跳与 ROS 等待空档，不启动任何仿真服务。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from follow_demo.executor_pulse import ExecutorPulse


class MeasuredExecutor(SingleThreadedExecutor):
    """记录独立唤醒线程调用 wake 的真实起止时间，不改变执行器行为。"""

    def __init__(self):
        """先准备记录容器，兼容父类初始化期间可能发生的唤醒。"""
        self.wake_calls = []
        super().__init__()

    def wake(self):
        """测量 guard condition 调用耗时；业务回调仍由唯一 spin 线程执行。"""
        started = time.monotonic()
        error = ''
        try:
            return super().wake()
        except Exception as failure:
            error = repr(failure)
            raise
        finally:
            self.wake_calls.append(dict(start=started, end=time.monotonic(), error=error))


class TimerProbe:
    """一个独立节点、单调时钟定时器和执行器组成一组对照。"""

    def __init__(self, mode, period):
        """两组保留相同 timer 和回调，只让 steady 组增加外部 guard 唤醒。"""
        self.mode, self.period = mode, period
        self.calls, self.errors = [], []
        self.node = Node('go2_executor_clock_' + mode + '_probe')
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.node.create_timer(period, self.record, clock=self.clock)
        self.executor = MeasuredExecutor() if mode == 'steady' else SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.spin, name='executor-clock-' + mode)

    def record(self):
        """只保存真实 callback 入口的单调时间，不发布消息或刷新其他状态。"""
        self.calls.append(time.monotonic())

    def spin(self):
        """在本组唯一线程中调度回调，并保留意外退出原因。"""
        try:
            self.executor.spin()
        except Exception as failure:
            self.errors.append(repr(failure))


class ClockSamples:
    """独立采样墙钟与单调时间的偏移，不依赖 ROS timer 调度。"""

    def __init__(self, period):
        """准备原始采样及可检查的回收信号，采样线程没有业务锁。"""
        self.period, self.samples, self.errors = period, [], []
        self.quit = threading.Event()
        self.thread = threading.Thread(target=self.run, name='executor-clock-sampler')

    def capture(self):
        """用两次单调取时夹住墙钟取时，标明每次采样的时间不确定区间。"""
        before = time.monotonic()
        system = time.time()
        after = time.monotonic()
        monotonic = (before + after) / 2
        self.samples.append(dict(monotonic=monotonic, realtime=system,
                                 offset=system - monotonic, capture_span_s=after - before))

    def run(self):
        """按单调期限采样；错过期限时跳过积压，避免人为制造高频补采。"""
        try:
            self.capture()
            deadline = time.monotonic() + self.period
            while not self.quit.wait(max(0., deadline - time.monotonic())):
                self.capture()
                now = time.monotonic()
                deadline += self.period
                if deadline <= now:
                    deadline = now + self.period
        except Exception as failure:
            self.errors.append(repr(failure))

    def close(self):
        """停止并回收本探针唯一采样线程，保留是否真正退出的证据。"""
        self.quit.set()
        self.thread.join(timeout=5)
        return not self.thread.is_alive()


def summarize_intervals(times, large_gap=.1):
    """统一汇总 callback、wake 或采样的实际间隔，保留全部大空档。"""
    gaps = [(end - start, start, end) for start, end in zip(times, times[1:])]
    ordered = sorted(gap for gap, _, _ in gaps)
    return dict(count=len(times), interval_count=len(gaps),
                maximum_gap_s=max(ordered, default=None),
                mean_gap_s=sum(ordered) / len(ordered) if ordered else None,
                p99_gap_s=ordered[min(len(ordered) - 1, int(len(ordered) * .99))] if ordered else None,
                large_gaps=[dict(gap_s=gap, before=start, after=end)
                            for gap, start, end in gaps if gap >= large_gap])


def jump_window(times, jump):
    """同时报告跳变前后最近回调，以及覆盖潜在延长等待区间的最大空档。"""
    moment = jump['monotonic']
    before = [value for value in times if value <= moment]
    after = [value for value in times if value > moment]
    # 负跳变可能将等待截止日期推迟相同秒数；正跳变仍保留短窗口供审查。
    window_start, window_end = moment - .1, moment + max(.5, -jump['delta_s'] + .5)
    intervals = [(end - start, start, end) for start, end in zip(times, times[1:])
                 if end >= window_start and start <= window_end]
    largest = max(intervals, default=None)
    return dict(window_start=window_start, window_end=window_end,
                last_before=before[-1] if before else None,
                first_after=after[0] if after else None,
                bracketing_gap_s=after[0] - before[-1] if before and after else None,
                maximum_overlap_gap=None if largest is None else
                dict(gap_s=largest[0], before=largest[1], after=largest[2]))


def build_report(started, ended, probes, sampler, pulse, cleanup, arguments, error, source_hashes):
    """保留实验身份、原始序列及跳变窗口，不把没有刺激的运行伪称为通过。"""
    timer_times = {probe.mode: [value for value in probe.calls if started <= value <= ended]
                   for probe in probes}
    wake_calls = [call for call in probes[1].executor.wake_calls
                  if started <= call['start'] <= ended]
    clock_samples = [row for row in sampler.samples if started <= row['monotonic'] <= ended]
    jumps = []
    for previous, current in zip(clock_samples, clock_samples[1:]):
        delta = current['offset'] - previous['offset']
        if abs(delta) > arguments.jump_threshold:
            jumps.append(dict(current, previous_monotonic=previous['monotonic'],
                              previous_realtime=previous['realtime'], delta_s=delta))
    windows = [dict(jump=jump,
                    timers={mode: jump_window(times, jump) for mode, times in timer_times.items()},
                    wake=jump_window([call['start'] for call in wake_calls], jump)) for jump in jumps]
    stats = {mode: summarize_intervals(times) for mode, times in timer_times.items()}
    negative_windows = [window for window in windows if window['jump']['delta_s'] < 0]
    # 只有实际回跳覆盖充分、原生组出现对应长等待且 B 组消除，才支持因果解释。
    demonstrated = [window for window in negative_windows
                    if (window['timers']['native']['bracketing_gap_s'] or 0) >=
                    max(.5, -.7 * window['jump']['delta_s'])
                    and (window['timers']['steady']['bracketing_gap_s'] or float('inf')) < .1]
    run_errors = [failure for probe in probes for failure in probe.errors] + sampler.errors
    if error or run_errors or pulse.diagnostics()['error'] or not all(cleanup.values()):
        conclusion = 'INCOMPLETE'
    elif len(negative_windows) >= 3 and demonstrated:
        conclusion = 'SUPPORTED_BY_AB'
    else:
        conclusion = 'NOT_DEMONSTRATED'
    return dict(schema_version=1, experiment='same_process_executor_clock_ab',
                started_at_utc=datetime.fromtimestamp(clock_samples[0]['realtime'], timezone.utc).isoformat()
                if clock_samples else None, domain_id=os.environ.get('ROS_DOMAIN_ID'),
                middleware=os.environ.get('RMW_IMPLEMENTATION'), requested_duration_s=arguments.duration,
                actual_duration_s=ended - started, timer_period_s=arguments.timer_period,
                pulse_period_s=arguments.pulse_period, sampler_period_s=arguments.sample_period,
                jump_threshold_s=arguments.jump_threshold, modes=['native', 'steady'],
                timer_clock='STEADY_TIME', clock_jump_count=len(jumps),
                negative_clock_jump_count=len(negative_windows), demonstrated_jump_count=len(demonstrated),
                sufficient_negative_jumps=len(negative_windows) >= 3, conclusion=conclusion,
                timers=stats, wake=dict(summarize_intervals([call['start'] for call in wake_calls]),
                                       maximum_call_s=max((call['end'] - call['start']
                                                           for call in wake_calls), default=None),
                                       errors=[call['error'] for call in wake_calls if call['error']]),
                sampler=summarize_intervals([row['monotonic'] for row in clock_samples]),
                pulse=pulse.diagnostics(), jump_windows=windows, cleanup=cleanup,
                error=error, thread_errors=run_errors,
                source_sha256=source_hashes,
                raw=dict(timer_calls=timer_times, wake_calls=wake_calls, clock_samples=clock_samples))


def main():
    """运行有时限的独立对照探针，结束时先停外部唤醒再释放 ROS 资源。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--timer-period', type=float, default=.01)
    parser.add_argument('--pulse-period', type=float, default=.02)
    parser.add_argument('--sample-period', type=float, default=.01)
    parser.add_argument('--jump-threshold', type=float, default=.05)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2] /
                        'logs' / 'executor-clock-ab-20261005.json')
    arguments = parser.parse_args()
    if any(value <= 0 for value in (arguments.duration, arguments.timer_period,
                                   arguments.pulse_period, arguments.sample_period,
                                   arguments.jump_threshold)):
        parser.error('时长、周期和跳变门槛必须为正')
    if not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('请通过命令环境显式设置独立 ROS_DOMAIN_ID')
    sources = [Path(__file__), Path(__file__).resolve().parents[1] / 'executor_pulse.py']
    # 在运行之前固定来源身份，避免其他并行任务更新文件后把报告错误归入新版本。
    source_hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    rclpy.init()
    probes = [TimerProbe(mode, arguments.timer_period) for mode in ('native', 'steady')]
    sampler = ClockSamples(arguments.sample_period)
    pulse = ExecutorPulse([probes[1].executor], period=arguments.pulse_period)
    cleanup, error = {}, ''
    started = time.monotonic()
    try:
        for probe in probes:
            probe.thread.start()
        sampler.thread.start()
        pulse.start()
        print(json.dumps(dict(event='started', pid=os.getpid(), domain_id=os.environ['ROS_DOMAIN_ID'],
                              duration_s=arguments.duration), ensure_ascii=False), flush=True)
        next_progress = started + 30
        while time.monotonic() - started < arguments.duration:
            time.sleep(min(.2, max(0., started + arguments.duration - time.monotonic())))
            now = time.monotonic()
            if now >= next_progress:
                print(json.dumps(dict(event='progress', elapsed_s=now - started,
                                      callbacks={probe.mode: len(probe.calls) for probe in probes},
                                      clock_jumps=sum(abs(current['offset'] - previous['offset']) >
                                                      arguments.jump_threshold for previous, current in
                                                      zip(sampler.samples, sampler.samples[1:])),
                                      pulse=pulse.diagnostics()), ensure_ascii=False), flush=True)
                next_progress += 30
    except BaseException as failure:
        error = repr(failure)
    finally:
        ended = time.monotonic()
        # guard condition 必须活得比唤醒线程久，避免清理时访问已销毁的 ROS 对象。
        try:
            pulse.close()
            cleanup['pulse_joined'] = not pulse.thread.is_alive()
        except Exception as failure:
            cleanup['pulse_joined'] = False
            error += '; pulse close: ' + repr(failure)
        cleanup['sampler_joined'] = sampler.close()
        for probe in probes:
            cleanup[probe.mode + '_executor_shutdown'] = probe.executor.shutdown(timeout_sec=5)
            probe.thread.join(timeout=5)
            cleanup[probe.mode + '_thread_joined'] = not probe.thread.is_alive()
        for probe in probes:
            probe.node.destroy_node()
        rclpy.shutdown()
        cleanup['ros_context_shutdown'] = not rclpy.ok()
    report = build_report(started, ended, probes, sampler, pulse, cleanup, arguments, error, source_hashes)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(event='complete', output=str(arguments.output),
                          conclusion=report['conclusion'], jumps=report['clock_jump_count'],
                          native=report['timers']['native'], steady=report['timers']['steady'],
                          wake=report['wake'], cleanup=cleanup), ensure_ascii=False), flush=True)
    return 1 if report['conclusion'] == 'INCOMPLETE' else 0


if __name__ == '__main__':
    raise SystemExit(main())
