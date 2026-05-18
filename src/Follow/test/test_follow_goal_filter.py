from go2_dynamic_follow_avoidance.target_filter import TargetFilter, TargetFilterConfig


def test_target_filter_requires_consecutive_valid_samples():
    filt = TargetFilter(TargetFilterConfig(min_valid_samples=3, smoothing_alpha=1.0))

    assert not filt.update((1.0, 0.0), True, 0.0).tracking
    assert not filt.update((1.0, 0.1), True, 0.1).tracking
    result = filt.update((1.0, 0.2), True, 0.2)

    assert result.tracking
    assert result.xy == (1.0, 0.2)


def test_target_filter_rejects_large_jump_and_holds_briefly():
    filt = TargetFilter(
        TargetFilterConfig(
            min_valid_samples=1,
            max_invalid_samples=2,
            hold_sec=0.5,
            smoothing_alpha=1.0,
            max_jump_m=0.5,
        )
    )

    assert filt.update((1.0, 0.0), True, 0.0).tracking
    held = filt.update((3.0, 0.0), True, 0.1)
    assert held.tracking
    assert held.xy == (1.0, 0.0)
    assert held.status.startswith("hold: jump")

    expired = filt.current(0.7, sample_timeout_sec=1.0)
    assert not expired.tracking


def test_target_filter_hard_stale_on_missing_uwb_samples():
    filt = TargetFilter(TargetFilterConfig(min_valid_samples=1, smoothing_alpha=1.0))

    assert filt.update((1.0, 0.0), True, 0.0).tracking
    stale = filt.current(0.4, sample_timeout_sec=0.3)

    assert not stale.tracking
    assert stale.status == "stale: UWB timeout"


def test_target_filter_invalid_sample_resets_acquisition_count():
    filt = TargetFilter(TargetFilterConfig(min_valid_samples=2, smoothing_alpha=1.0))

    assert not filt.update((1.0, 0.0), True, 0.0).tracking
    assert not filt.update(None, False, 0.1, "invalid").tracking
    assert not filt.update((1.0, 0.0), True, 0.2).tracking
    assert filt.update((1.0, 0.0), True, 0.3).tracking
