from go2_dynamic_follow_avoidance.grid_planner import GridSpec, astar, build_costmap, clear_radius


def test_astar_routes_around_blocked_column():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    start = spec.world_to_cell((0.0, 0.0))
    goal = spec.world_to_cell((1.2, 0.0))
    blocked = set()
    for y in range(0, spec.height_cells):
        if y == spec.world_to_cell((0.5, 0.8))[1]:
            continue
        blocked.add((spec.world_to_cell((0.5, 0.0))[0], y))

    blocked = clear_radius(spec, blocked, (0.0, 0.0), 0.2)
    path = astar(spec, start, goal, blocked)

    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert all(cell not in blocked for cell in path)


def test_costmap_front_obstacle_distance():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    result = build_costmap(
        spec,
        [(0.4, 0.0, 0.3), (1.4, 0.8, 0.3), (0.2, 0.8, 2.0)],
        obstacle_x_min=0.05,
        obstacle_x_max=2.0,
        obstacle_y_abs=1.0,
        obstacle_z_min=0.05,
        obstacle_z_max=1.2,
        inflation_radius=0.2,
        emergency_x_max=0.45,
        emergency_y_abs=0.45,
        max_points=100,
    )

    assert result.nearest_front_obstacle_m == 0.4
    assert len(result.occupied) == 2
    assert len(result.inflated) >= len(result.occupied)
