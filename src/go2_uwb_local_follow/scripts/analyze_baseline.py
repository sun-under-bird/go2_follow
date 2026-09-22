#!/usr/bin/env python3
# Copyright 2026 OpenAI
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Export recorded control cycles to CSV and comparable baseline metrics to JSON."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


def statistics(values):
    """Summarize finite samples without treating missing data as zero."""
    samples = sorted(v for v in values if math.isfinite(v))
    if not samples:
        return {'count': 0}

    def percentile(p):
        index = (len(samples) - 1) * p
        lo = int(index)
        hi = min(lo + 1, len(samples) - 1)
        return samples[lo] + (samples[hi] - samples[lo]) * (index - lo)

    return {'count': len(samples), 'mean': sum(samples) / len(samples),
            'min': samples[0], 'p50': percentile(0.5), 'p95': percentile(0.95),
            'p99': percentile(0.99), 'max': samples[-1]}


def numeric(row, name):
    """Parse optional diagnostic numbers."""
    try:
        return float(row.get(name, 'nan'))
    except ValueError:
        return math.nan


def summarize(rows, follow_distance, period_ms):
    """Summarize timing, distance estimates and measured motion transitions."""
    report = {'cycles': len(rows), 'follow_distance_m': follow_distance,
              'control_period_ms': period_ms}
    for key in ('planning_time_ms', 'compensation_ms', 'previous_cycle_total_ms',
                'control_interval_ms', 'command_latency_ms', 'obstacle_source_age_sec',
                'obstacle_receipt_age_sec', 'odom_source_age_sec', 'min_clearance'):
        samples = [numeric(r, key) for r in rows]
        if key == 'previous_cycle_total_ms':
            samples = [numeric(r, key) for r in rows
                       if numeric(r, 'previous_control_stamp_ns') > 0]
        report[key] = statistics(samples)
    errors = [numeric(r, 'distance') - follow_distance for r in rows
              if numeric(r, 'target_source_stamp_ns') > 0
              and numeric(r, 'target_age_sec') <= 0.5]
    report['distance_absolute_error_m'] = statistics([abs(e) for e in errors])
    finite_errors = [e for e in errors if math.isfinite(e)]
    report['distance_rmse_m'] = (math.sqrt(sum(e * e for e in finite_errors) / len(finite_errors))
                                 if finite_errors else None)
    report['cycle_deadline_exceeded'] = sum(
        numeric(r, 'previous_cycle_total_ms') > period_ms for r in rows)
    report['cycle_sequence_gaps'] = sum(
        max(0, int(numeric(b, 'cycle_sequence') - numeric(a, 'cycle_sequence') - 1))
        for a, b in zip(rows, rows[1:])
        if math.isfinite(numeric(a, 'cycle_sequence'))
        and math.isfinite(numeric(b, 'cycle_sequence')))
    states = Counter()
    starts = stops = reverses = 0
    moving = None
    pending = None
    pending_since = 0.0
    previous_state = None
    for row in rows:
        state = row['state']
        if state != previous_state:
            states[state] += 1
            reverses += int(state == 'EMERGENCY_REVERSING')
        previous_state = state
        speed = numeric(row, 'measured_v')
        if numeric(row, 'odom_age_sec') > 0.1 or not math.isfinite(speed):
            pending = None
            continue
        desired = True if abs(speed) >= 0.10 else False if abs(speed) <= 0.04 else moving
        stamp = numeric(row, 'control_stamp_ns') * 1e-9
        if desired != pending:
            pending, pending_since = desired, stamp
        if pending is not None and pending != moving and stamp - pending_since >= 0.20:
            if moving is not None:
                starts += int(pending)
                stops += int(not pending)
            moving = pending
    report.update(state_entries=dict(states), measured_starts=starts, measured_stops=stops,
                  reverse_entries=reverses)
    report['limitations'] = [
        'Distance is the UWB estimate, not independent ground truth.',
        'min_clearance is predicted clearance, not measured physical side clearance.',
        'previous_cycle_total_ms includes publication and telemetry; '
        'its stamp identifies the prior cycle.',
        'Sequence gaps detect missing telemetry, not all sensor/recorder losses; '
        'inspect recorder.log.',
        'Starts/stops use measured |v| thresholds 0.10/0.04 m/s sustained for 0.20 s.',
    ]
    return report


def main():
    """Read a ROS 2 bag without publishing any recorded messages."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--follow-distance', type=float, default=1.0)
    parser.add_argument('--control-period-ms', type=float, default=50.0)
    args = parser.parse_args()
    import rosbag2_py
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    rows = []
    counts = Counter()
    timing = {}
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        counts[topic] += 1
        timing.setdefault(topic, [stamp, stamp])[1] = stamp
        if topic != '/go2_uwb_local_follow/control_cycle':
            continue
        message = deserialize_message(payload, DiagnosticArray)
        for status in message.status:
            row = {item.key: item.value for item in status.values}
            row.update(state=status.message, bag_stamp_ns=stamp)
            rows.append(row)
    if not rows:
        raise SystemExit('No control_cycle telemetry in bag; enable_cycle_telemetry must be true.')
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'cycles.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted({k for r in rows for k in r}))
        writer.writeheader()
        writer.writerows(rows)
    report = summarize(rows, args.follow_distance, args.control_period_ms)
    report['topic_counts'] = dict(counts)
    report['topic_recorded_hz'] = {
        topic: (counts[topic] - 1) / ((last - first) * 1e-9)
        for topic, (first, last) in timing.items() if last > first}
    (args.output / 'summary.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
