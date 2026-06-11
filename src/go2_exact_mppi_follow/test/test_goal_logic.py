from go2_exact_mppi_follow.goal_logic import (
    FollowGoalConfig,
    UwbParseConfig,
    compute_follow_goal,
    parse_uwb_target,
)


def test_parse_uwb_prefers_xy():
    """Check that x/y fields are preferred when they are valid."""

    target = parse_uwb_target(2.0, 0.5, 9.0, 1.0, UwbParseConfig())
    assert target == (2.0, 0.5)


def test_parse_uwb_falls_back_to_range_angle():
    """Check range-angle parsing when x/y is unavailable."""

    target = parse_uwb_target(0.0, 0.0, 2.0, 0.0, UwbParseConfig())
    assert target == (2.0, 0.0)


def test_follow_goal_keeps_distance_gap():
    """Check that the goal is shortened by the follow distance."""

    goal = compute_follow_goal((2.0, 0.0), FollowGoalConfig(follow_distance=0.9))
    assert goal is not None
    assert abs(goal[0] - 1.1) < 1e-6
    assert abs(goal[1]) < 1e-6


def test_follow_goal_clamps_reverse_motion():
    """Check that close targets only request limited reverse motion."""

    goal = compute_follow_goal((0.2, 0.0), FollowGoalConfig(follow_distance=0.9, reverse_goal_limit=0.4))
    assert goal is not None
    assert abs(goal[0] + 0.4) < 1e-6
