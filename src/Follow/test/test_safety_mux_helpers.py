from go2_dynamic_follow_avoidance.safety_helpers import limit_delta


def test_limit_delta_limits_positive_and_negative_steps():
    assert limit_delta(0.0, 1.0, 0.2) == 0.2
    assert limit_delta(0.5, -0.5, 0.3) == 0.2
    assert limit_delta(0.5, 0.6, 0.3) == 0.6
