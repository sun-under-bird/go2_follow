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

"""Check missing-data handling and loss detection in baseline reports."""

import importlib.util
import math
from pathlib import Path


def load_analysis():
    """Load the installed-independent report helpers."""
    path = Path(__file__).resolve().parents[1] / 'scripts' / 'analyze_baseline.py'
    spec = importlib.util.spec_from_file_location('analysis', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_samples_are_not_reported_as_zero():
    """Missing or invalid distances must not fabricate perfect tracking."""
    module = load_analysis()
    assert module.statistics([math.inf, math.nan]) == {'count': 0}
    result = module.summarize([{'state': 'WAIT_TARGET'}], 1.0, 50.0)
    assert result['distance_absolute_error_m']['count'] == 0
    assert result['distance_rmse_m'] is None


def test_deadline_and_sequence_loss_are_reported():
    """Detect a missed cycle budget and dropped telemetry independently."""
    module = load_analysis()
    rows = [dict(state='PLANNING', cycle_sequence=str(i), previous_cycle_total_ms=str(ms),
                 previous_control_stamp_ns='100', target_source_stamp_ns='99',
                 target_age_sec='0.1', distance='1.2') for i, ms in [(1, 10), (4, 60)]]
    result = module.summarize(rows, 1.0, 50.0)
    assert result['cycle_sequence_gaps'] == 2
    assert result['cycle_deadline_exceeded'] == 1
    assert abs(result['distance_rmse_m'] - 0.2) < 1e-9
