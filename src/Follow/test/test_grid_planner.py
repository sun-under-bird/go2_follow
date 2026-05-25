from go2_dynamic_follow_avoidance.grid_planner import (
    GridSpec,
    astar,
    build_costmap,
    clear_radius,
    point_cells,
    project_goal_to_grid,
    raytrace_cells,
)


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


def test_costmap_requires_multiple_points_per_cell():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    result = build_costmap(
        spec,
        [(0.4, 0.0, 0.3), (0.42, 0.01, 0.3), (1.0, 0.6, 0.3)],
        obstacle_x_min=0.05,
        obstacle_x_max=2.0,
        obstacle_y_abs=1.0,
        obstacle_z_min=0.05,
        obstacle_z_max=1.2,
        inflation_radius=0.2,
        emergency_x_max=0.45,
        emergency_y_abs=0.45,
        max_points=100,
        min_points_per_cell=2,
    )

    assert result.nearest_front_obstacle_m == 0.4
    assert len(result.occupied) == 1


def test_point_cells_filters_ground_clear_points():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    cells = point_cells(
        spec,
        [(0.4, 0.0, 0.05), (0.42, 0.01, 0.08), (0.5, 0.0, 0.4), (1.0, 1.3, 0.05)],
        x_min=0.05,
        x_max=2.0,
        y_abs=1.0,
        z_min=0.0,
        z_max=0.25,
        max_points=100,
        min_points_per_cell=2,
    )

    assert cells == {spec.world_to_cell((0.4, 0.0))}


def test_raytrace_cells_clears_between_base_and_ground():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    cells = raytrace_cells(spec, (0.0, 0.0), [(0.6, 0.0)])

    assert spec.world_to_cell((0.0, 0.0)) in cells
    assert spec.world_to_cell((0.3, 0.0)) in cells
    assert spec.world_to_cell((0.6, 0.0)) in cells


def test_project_goal_to_grid_clips_to_front_edge():
    spec = GridSpec(width_m=2.0, height_m=2.0, resolution=0.1, origin_x=-0.5, origin_y=-1.0)
    projected = project_goal_to_grid(spec, (3.0, 0.6))

    assert projected is not None
    assert spec.world_to_cell(projected) is not None
    assert projected[0] < 1.5
