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

"""Record a passive baseline bag, runtime parameters and reproducibility manifest."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time


PREFIX = '/go2_uwb_local_follow/'
TOPICS = [
    '/libAoa_robot_publisher', '/uwb/target_point', '/odom_leg',
    '/local_grid_obstacle', '/local_depth_observation', '/local_rolling_obstacle',
    '/cmd_vel', '/tf', '/tf_static', '/clock', '/parameter_events',
    '/uwb/target_adapter_diagnostics', '/stereo/obstacle_diagnostics',
] + [PREFIX + name for name in (
    'nominal_cmd', 'planned_cmd', 'final_cmd', 'selected_path', 'control_obstacles',
    'follow_diagnostics', 'planner_diagnostics', 'rolling_map_diagnostics',
    'control_cycle', 'follow_cycle', 'map_observations', 'state_transitions',
)]
NODES = [
    '/libAoa_robot_publisher', '/disparity_node', '/stereo_obstacle_projector_node',
    '/uwb_target_adapter_node', '/uwb_follow_controller_node',
    '/rolling_obstacle_map_node', '/local_velocity_planner_node',
]


def capture(command, cwd=None):
    """Capture read-only CLI metadata with a bounded wait and explicit failure."""
    try:
        result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, timeout=15)
        return {'command': command, 'returncode': result.returncode,
                'stdout': result.stdout, 'stderr': result.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'command': command, 'error': str(exc)}


def main():
    """Record until duration elapses or the operator interrupts; never publish commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New experiment directory')
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--duration', type=float, default=60.0, help='Seconds; 0 until Ctrl-C')
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--notes', default='',
                        help='Launch command, scene dimensions, operator notes')
    parser.add_argument('--include-images', action='store_true')
    parser.add_argument('--topic', action='append', default=[], help='Additional/remapped topic')
    parser.add_argument('--node', action='append', default=[], help='Additional parameter source')
    args = parser.parse_args()
    if args.duration < 0:
        parser.error('--duration must be nonnegative')
    args.output.mkdir(parents=True, exist_ok=False)
    parameters = args.output / 'parameters'
    parameters.mkdir()
    topics = list(dict.fromkeys(TOPICS + args.topic))
    if args.include_images:
        topics += ['/stereo/disparity'] + [
            '/camera/camera/' + camera + '/' + suffix
            for camera in ('infra1', 'infra2') for suffix in ('image_rect_raw', 'camera_info')]
    manifest = {
        'scenario': args.scenario, 'notes': args.notes,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'duration_requested_sec': args.duration, 'topics': topics,
        'ros_distro': os.getenv('ROS_DISTRO'), 'rmw': os.getenv('RMW_IMPLEMENTATION'),
        'ros_domain_id': os.getenv('ROS_DOMAIN_ID', '0'),
        'git': capture(['git', 'rev-parse', 'HEAD'], args.repo),
        'git_status': capture(['git', 'status', '--short'], args.repo),
        'topic_graph': capture(['ros2', 'topic', 'list', '-t', '--no-daemon']),
        'parameter_capture': {},
    }
    diff = capture(['git', 'diff', 'HEAD'], args.repo)
    (args.output / 'working_tree.patch').write_text(diff.get('stdout', ''), encoding='utf-8')
    for node in dict.fromkeys(NODES + args.node):
        result = capture(['ros2', 'param', 'dump', node, '--no-daemon'])
        manifest['parameter_capture'][node] = result
        if result.get('returncode') == 0:
            (parameters / (node.strip('/').replace('/', '_') + '.yaml')).write_text(
                result['stdout'], encoding='utf-8')
    qos = args.output / 'record_qos.yaml'
    qos.write_text('/tf_static:\n  reliability: reliable\n  durability: transient_local\n'
                   '  history: keep_last\n  depth: 100\n', encoding='utf-8')
    command = ['ros2', 'bag', 'record', '-o', str(args.output / 'bag'),
               '--qos-profile-overrides-path', str(qos)] + topics
    manifest['record_command'] = command
    manifest_path = args.output / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Recording {args.scenario} into {args.output}; Ctrl-C stops recording.', flush=True)
    start = time.monotonic()
    with (args.output / 'recorder.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            while process.poll() is None:
                if args.duration and time.monotonic() - start >= args.duration:
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            manifest['record_elapsed_sec'] = time.monotonic() - start
            manifest['recorder_returncode'] = process.returncode
            manifest['bag_info'] = capture(['ros2', 'bag', 'info', str(args.output / 'bag')])
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2),
                                     encoding='utf-8')
    if not (args.output / 'bag' / 'metadata.yaml').exists():
        raise SystemExit('Recording failed: inspect recorder.log and manifest.json')


if __name__ == '__main__':
    main()
