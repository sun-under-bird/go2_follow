import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Cell = Tuple[int, int]
Point2D = Tuple[float, float]


@dataclass
class GridSpec:
    width_m: float
    height_m: float
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width_cells(self) -> int:
        return int(round(self.width_m / self.resolution))

    @property
    def height_cells(self) -> int:
        return int(round(self.height_m / self.resolution))

    def contains_cell(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width_cells and 0 <= y < self.height_cells

    def world_to_cell(self, point: Point2D) -> Optional[Cell]:
        x, y = point
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        cell = (cx, cy)
        if not self.contains_cell(cell):
            return None
        return cell

    def cell_to_world(self, cell: Cell) -> Point2D:
        cx, cy = cell
        return (
            self.origin_x + (cx + 0.5) * self.resolution,
            self.origin_y + (cy + 0.5) * self.resolution,
        )


@dataclass
class CostmapBuildResult:
    occupied: Set[Cell]
    inflated: Set[Cell]
    nearest_front_obstacle_m: Optional[float]


def build_costmap(
    spec: GridSpec,
    points: Iterable[Tuple[float, float, float]],
    obstacle_x_min: float,
    obstacle_x_max: float,
    obstacle_y_abs: float,
    obstacle_z_min: float,
    obstacle_z_max: float,
    inflation_radius: float,
    emergency_x_max: float,
    emergency_y_abs: float,
    max_points: int,
    min_points_per_cell: int = 1,
) -> CostmapBuildResult:
    points_list = points if isinstance(points, list) else list(points)
    nearest_front: Optional[float] = None
    seen = 0

    for px, py, pz in points_list:
        if seen >= max_points:
            break
        seen += 1
        if not (obstacle_x_min <= px <= obstacle_x_max):
            continue
        if abs(py) > obstacle_y_abs:
            continue
        if not (obstacle_z_min <= pz <= obstacle_z_max):
            continue

        if 0.0 <= px <= emergency_x_max and abs(py) <= emergency_y_abs:
            nearest_front = px if nearest_front is None else min(nearest_front, px)

    occupied = point_cells(
        spec,
        points_list,
        obstacle_x_min,
        obstacle_x_max,
        obstacle_y_abs,
        obstacle_z_min,
        obstacle_z_max,
        max_points,
        min_points_per_cell,
    )
    inflated = inflate_cells(spec, occupied, inflation_radius)
    return CostmapBuildResult(occupied=occupied, inflated=inflated, nearest_front_obstacle_m=nearest_front)


def point_cells(
    spec: GridSpec,
    points: Iterable[Tuple[float, float, float]],
    x_min: float,
    x_max: float,
    y_abs: float,
    z_min: float,
    z_max: float,
    max_points: int,
    min_points_per_cell: int = 1,
) -> Set[Cell]:
    cell_counts: Dict[Cell, int] = {}
    seen = 0
    min_points_per_cell = max(1, int(min_points_per_cell))

    for px, py, pz in points:
        if seen >= max_points:
            break
        seen += 1
        if not (x_min <= px <= x_max):
            continue
        if abs(py) > y_abs:
            continue
        if not (z_min <= pz <= z_max):
            continue

        cell = spec.world_to_cell((px, py))
        if cell is not None:
            cell_counts[cell] = cell_counts.get(cell, 0) + 1

    return {cell for cell, count in cell_counts.items() if count >= min_points_per_cell}


def raytrace_cells(spec: GridSpec, start: Point2D, ends: Iterable[Point2D]) -> Set[Cell]:
    start_cell = spec.world_to_cell(start)
    if start_cell is None:
        return set()

    traced: Set[Cell] = set()
    for end in ends:
        end_cell = spec.world_to_cell(end)
        if end_cell is None:
            continue
        traced.update(_bresenham_cells(spec, start_cell, end_cell))
    return traced


def _bresenham_cells(spec: GridSpec, start: Cell, end: Cell) -> Set[Cell]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    cells: Set[Cell] = set()

    while True:
        if spec.contains_cell((x, y)):
            cells.add((x, y))
        if x == x1 and y == y1:
            break
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x += sx
        if err2 < dx:
            err += dx
            y += sy
    return cells


def inflate_cells(spec: GridSpec, occupied: Set[Cell], radius_m: float) -> Set[Cell]:
    if radius_m <= 0.0:
        return set(occupied)

    radius_cells = int(math.ceil(radius_m / spec.resolution))
    inflated = set(occupied)
    radius_sq = radius_m * radius_m
    for ox, oy in occupied:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                wx = dx * spec.resolution
                wy = dy * spec.resolution
                if wx * wx + wy * wy > radius_sq:
                    continue
                cell = (ox + dx, oy + dy)
                if spec.contains_cell(cell):
                    inflated.add(cell)
    return inflated


def clear_radius(spec: GridSpec, blocked: Set[Cell], center: Point2D, radius_m: float) -> Set[Cell]:
    center_cell = spec.world_to_cell(center)
    if center_cell is None or radius_m <= 0.0:
        return blocked

    result = set(blocked)
    radius_cells = int(math.ceil(radius_m / spec.resolution))
    radius_sq = radius_m * radius_m
    cx, cy = center_cell
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            wx = dx * spec.resolution
            wy = dy * spec.resolution
            if wx * wx + wy * wy <= radius_sq:
                result.discard((cx + dx, cy + dy))
    return result


def nearest_free_cell(spec: GridSpec, blocked: Set[Cell], desired: Cell, max_radius_m: float) -> Optional[Cell]:
    if spec.contains_cell(desired) and desired not in blocked:
        return desired

    max_radius_cells = int(math.ceil(max_radius_m / spec.resolution))
    best: Optional[Cell] = None
    best_dist = float("inf")
    dx0, dy0 = desired
    for radius in range(1, max_radius_cells + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) != radius and abs(dy) != radius:
                    continue
                cell = (dx0 + dx, dy0 + dy)
                if not spec.contains_cell(cell) or cell in blocked:
                    continue
                dist = math.hypot(dx, dy)
                if dist < best_dist:
                    best = cell
                    best_dist = dist
        if best is not None:
            return best
    return None


def project_goal_to_grid(spec: GridSpec, goal: Point2D) -> Optional[Point2D]:
    """Project an out-of-bounds goal onto the local grid along the base-to-goal ray."""
    gx, gy = goal
    if spec.world_to_cell(goal) is not None:
        return goal
    if abs(gx) + abs(gy) < 1e-9:
        return (0.0, 0.0) if spec.world_to_cell((0.0, 0.0)) is not None else None

    min_x = spec.origin_x + spec.resolution * 0.5
    max_x = spec.origin_x + spec.width_m - spec.resolution * 0.5
    min_y = spec.origin_y + spec.resolution * 0.5
    max_y = spec.origin_y + spec.height_m - spec.resolution * 0.5
    candidates = []

    if gx > 0.0:
        candidates.append(max_x / gx)
    elif gx < 0.0:
        candidates.append(min_x / gx)
    if gy > 0.0:
        candidates.append(max_y / gy)
    elif gy < 0.0:
        candidates.append(min_y / gy)

    candidates = [scale for scale in candidates if 0.0 < scale <= 1.0]
    if not candidates:
        return None

    scale = min(candidates)
    projected = (gx * scale, gy * scale)
    return projected if spec.world_to_cell(projected) is not None else None


def astar(spec: GridSpec, start: Cell, goal: Cell, blocked: Set[Cell]) -> Optional[List[Cell]]:
    if not spec.contains_cell(start) or not spec.contains_cell(goal):
        return None
    if start in blocked or goal in blocked:
        return None

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    def heuristic(cell: Cell) -> float:
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    open_heap: List[Tuple[float, float, Cell]] = [(heuristic(start), 0.0, start)]
    came_from: Dict[Cell, Cell] = {}
    g_score: Dict[Cell, float] = {start: 0.0}
    closed: Set[Cell] = set()

    while open_heap:
        _, current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct_path(came_from, current)
        closed.add(current)

        for dx, dy, move_cost in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not spec.contains_cell(nxt) or nxt in blocked or nxt in closed:
                continue
            if dx != 0 and dy != 0:
                side_a = (current[0] + dx, current[1])
                side_b = (current[0], current[1] + dy)
                if side_a in blocked or side_b in blocked:
                    continue

            tentative = current_cost + move_cost
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nxt), tentative, nxt))

    return None


def reconstruct_path(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def cells_to_points(spec: GridSpec, cells: Sequence[Cell]) -> List[Point2D]:
    return [spec.cell_to_world(cell) for cell in cells]


def simplify_path(points: Sequence[Point2D]) -> List[Point2D]:
    if len(points) <= 2:
        return list(points)

    simplified = [points[0]]
    last_dir = None
    for i in range(1, len(points)):
        prev = points[i - 1]
        cur = points[i]
        dx = round(cur[0] - prev[0], 6)
        dy = round(cur[1] - prev[1], 6)
        norm = math.hypot(dx, dy)
        direction = (round(dx / norm, 3), round(dy / norm, 3)) if norm > 0 else (0.0, 0.0)
        if last_dir is not None and direction != last_dir:
            simplified.append(prev)
        last_dir = direction
    simplified.append(points[-1])
    return simplified


def resample_path(points: Sequence[Point2D], spacing: float) -> List[Point2D]:
    if len(points) <= 1 or spacing <= 0.0:
        return list(points)

    result = [points[0]]
    for start, end in zip(points, points[1:]):
        sx, sy = start
        ex, ey = end
        length = math.hypot(ex - sx, ey - sy)
        if length < 1e-6:
            continue
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(1, steps + 1):
            t = step / steps
            result.append((sx + (ex - sx) * t, sy + (ey - sy) * t))
    return result


def occupancy_grid_data(spec: GridSpec, occupied: Set[Cell], inflated: Set[Cell]) -> List[int]:
    data = [0] * (spec.width_cells * spec.height_cells)
    for x, y in inflated:
        if spec.contains_cell((x, y)):
            data[x + y * spec.width_cells] = 60
    for x, y in occupied:
        if spec.contains_cell((x, y)):
            data[x + y * spec.width_cells] = 100
    return data
