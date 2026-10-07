"""从传感器地图验证观察几何；预测未知格与实际地图变化采用相同世界格编号。"""
import math
from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from .footprint import polygon_cells


# 仅观察姿态使用的仿真工程余量，尚未由实机转向漂移测量标定。
OBSERVATION_DRIFT_MARGIN = .10
OBSERVATION_MIN_GAIN = .10
OBSERVATION_MIN_OVERLAP = .30
OBSERVATION_REGION_RADIUS = .35


def camera_ground_projection(points, intrinsic, rotation, translation, image_shape):
    """用真实针孔标定投影 z=0 地面；返回几何支持，绝不能作为自由空间测量。"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    intrinsic = np.asarray(intrinsic, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if (intrinsic.shape != (4,) or rotation.shape != (3, 3) or translation.shape != (3,)
            or not np.isfinite(intrinsic).all() or not np.isfinite(rotation).all()
            or not np.isfinite(translation).all() or intrinsic[0] <= 0 or intrinsic[1] <= 0):
        raise ValueError('相机内参或 optical 到地面坐标系外参无效')
    if len(image_shape) != 2 or any(int(value) != value or value < 3 for value in image_shape):
        raise ValueError('深度图尺寸必须为至少 3×3 的整数高宽')
    height, width = map(int, image_shape)
    fx, fy, cx, cy = intrinsic
    ground = np.column_stack((points, np.zeros(len(points))))
    optical = (ground - translation) @ rotation
    z = optical[:, 2]
    # 与 RollingMap.integrate 的 ground_inside 完全同口径，包括最近/最远距和 3×3 腐蚀边界。
    finite = np.isfinite(optical).all(axis=1)
    safe_z = np.where(finite, np.maximum(z, 1e-6), 1.)
    u = np.rint(np.where(finite, fx * optical[:, 0] / safe_z + cx, -1.)).astype(int)
    v = np.rint(np.where(finite, fy * optical[:, 1] / safe_z + cy, -1.)).astype(int)
    supported = (finite & (z > .3) & (z < 4.8) & (u >= 1) & (u < width - 1)
                 & (v >= 1) & (v < height - 1))
    return dict(supported=supported, u=u, v=v, z=z)


@dataclass(frozen=True)
class CameraGroundModel:
    """不可变的 CameraInfo 与 optical→base_footprint 标定；高度和俯仰来自真实采集 TF。"""
    intrinsic: tuple
    image_shape: tuple
    base_rotation: tuple
    base_translation: tuple

    def __post_init__(self):
        """复制为不可变元组并验证标定，后台搜索不得共享可被传感器线程改写的数组。"""
        intrinsic = tuple(map(float, self.intrinsic))
        shape = tuple(self.image_shape)
        rotation = np.asarray(self.base_rotation, dtype=float)
        translation = np.asarray(self.base_translation, dtype=float)
        camera_ground_projection([], intrinsic, rotation, translation, shape)
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-6)):
            raise ValueError('相机旋转必须为有效正交旋转矩阵')
        object.__setattr__(self, 'intrinsic', intrinsic)
        object.__setattr__(self, 'image_shape', tuple(map(int, shape)))
        object.__setattr__(self, 'base_rotation', tuple(map(tuple, rotation.tolist())))
        object.__setattr__(self, 'base_translation', tuple(map(float, translation)))

    @classmethod
    def from_capture(cls, intrinsic, image_shape, rotation, translation, body_point, heading):
        """将采集时刻 optical→odom TF 去除平面机身姿态，保留真实光心偏移、高度和倾斜。"""
        base = np.asarray(body_point, dtype=float)
        if base.shape != (2,) or not np.isfinite(base).all() or not math.isfinite(heading):
            raise ValueError('采集时刻的平面机身姿态无效')
        cosine, sine = math.cos(heading), math.sin(heading)
        yaw = np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])
        relative = yaw.T @ (np.asarray(translation, dtype=float) - np.r_[base, 0.])
        return cls(intrinsic, image_shape, yaw.T @ np.asarray(rotation, dtype=float), relative)

    def transform(self, point, heading):
        """为候选平面视点重建 optical→odom；当前倾斜仅是预测，实际准入仍须更新采集模型。"""
        point = np.asarray(point, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all() or not math.isfinite(heading):
            raise ValueError('候选机身位置与朝向无效')
        cosine, sine = math.cos(heading), math.sin(heading)
        yaw = np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])
        return (yaw @ np.asarray(self.base_rotation),
                yaw @ np.asarray(self.base_translation) + np.r_[point, 0.])


def camera_ground_unoccluded(grid, origin, points):
    """从实际光心批量追踪地面投影射线；未知只代表潜在收益，已知障碍与窗口外阻挡预测。"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    origin = np.asarray(origin, dtype=float)
    result = np.ones(len(points), dtype=bool)
    if not len(points):
        return result
    if origin.shape != (2,) or not np.isfinite(origin).all() or not np.isfinite(points).all():
        return np.zeros(len(points), dtype=bool)
    start = (origin - grid.origin) / grid.resolution
    finish = (points - grid.origin) / grid.resolution
    if np.any(start <= 0.) or np.any(start >= grid.size):
        return np.zeros(len(points), dtype=bool)
    end = np.floor(finish).astype(int)
    result &= np.all((end >= 0) & (end < grid.size), axis=1)
    # 光心恰在格边/角时，所有接触格都属于射线；不能靠方向舍弃起点旁的已知障碍。
    touches = [[math.floor(value)] for value in start]
    for axis in (0, 1):
        if abs(start[axis] - round(start[axis])) <= 1e-9:
            touches[axis] = [round(start[axis]) - 1, round(start[axis])]
    if any(index < 0 or index >= grid.size for indices in touches for index in indices):
        return np.zeros(len(points), dtype=bool)
    if any(grid.occupied[row, col] for col in touches[0] for row in touches[1]):
        return np.zeros(len(points), dtype=bool)
    if not np.any(grid.occupied):
        return result
    delta = finish - start
    steps = np.sign(delta).astype(int)
    cells = np.tile(np.floor(start).astype(int), (len(points), 1))
    for axis in (0, 1):
        if abs(start[axis] - round(start[axis])) <= 1e-9:
            cells[steps[:, axis] < 0, axis] -= 1
    # 二维占用没有障碍高度，保守视为遮挡；这不会预测矮障碍上方的额外可见空间。
    # DDA 在每条栅格边界推进；同时触角时额外核对两侧格，等间距采样会漏过毫米级贴角。
    tick = np.full_like(delta, np.inf)
    np.divide(1., np.abs(delta), out=tick, where=delta != 0.)
    boundary = cells + (steps > 0)
    next_cross = np.full_like(delta, np.inf)
    np.divide(boundary - start, delta, out=next_cross, where=delta != 0.)
    active = result.copy()
    for _ in range(2 * grid.size + 3):
        indices = np.flatnonzero(active)
        if not len(indices):
            break
        current = cells[indices]
        inside = np.all((current >= 0) & (current < grid.size), axis=1)
        blocked = ~inside
        good = indices[inside]
        blocked[inside] = grid.occupied[cells[good, 1], cells[good, 0]]
        result[indices[blocked]] = False
        active[indices[blocked]] = False
        done = np.all(cells[indices] == end[indices], axis=1)
        active[indices[done]] = False
        indices = np.flatnonzero(active)
        if not len(indices):
            break
        cross = next_cross[indices]
        corner = np.abs(cross[:, 0] - cross[:, 1]) <= 1e-10
        for axis in (0, 1):
            sides = indices[corner]
            if len(sides):
                side = cells[sides].copy()
                side[:, axis] += steps[sides, axis]
                inside = np.all((side >= 0) & (side < grid.size), axis=1)
                blocked = ~inside
                blocked[inside] = grid.occupied[side[inside, 1], side[inside, 0]]
                result[sides[blocked]] = False
                active[sides[blocked]] = False
        move_x = (cross[:, 0] < cross[:, 1]) | corner
        move_y = (cross[:, 1] < cross[:, 0]) | corner
        for axis, moves in ((0, move_x), (1, move_y)):
            moved = indices[moves]
            cells[moved, axis] += steps[moved, axis]
            next_cross[moved, axis] += tick[moved, axis]
    # 防御异常几何导致的未结束射线；不能把有预算上限的计算当作确认可见。
    result[active] = False
    return result


def camera_ground_view(grid, free, point, heading, camera_model, include_visible=True, selected_cells=None):
    """预测真实相机地面支持；可仅查询任务 ROI，缺少标定禁止回退机身锥。"""
    selected_world = None if selected_cells is None else set(map(tuple, selected_cells))
    selected_count = grid.size * grid.size if selected_world is None else len(selected_world)
    selected = (np.arange(grid.size * grid.size) if selected_world is None
                else np.flatnonzero(local_cell_mask(grid, selected_world)))
    # 窗口外相关格始终保留在 ROI 分母内，不能通过滚动窗口裁剪制造高覆盖率。
    known_count = int(np.count_nonzero((free | grid.occupied).ravel()[selected]))
    selected_unknown_count = selected_count - known_count
    if camera_model is None:
        return dict(available=False, reason='CAMERA_GEOMETRY_UNAVAILABLE', gain=0.,
                    unknown_cells=set(), visible_cells=set(), covered_count=0, projected_count=0,
                    selected_count=selected_count, selected_unknown_count=selected_unknown_count,
                    roi_unknown_coverage=0.)
    rotation, translation = camera_model.transform(point, heading)
    gx, gy = grid.centers()
    points = np.column_stack((gx.ravel(), gy.ravel()))
    # 未提供 ROI 才查询全地图；任务评分只需下一段包络缺失格，避免重复追踪无关远处空间。
    projection = camera_ground_projection(points[selected], camera_model.intrinsic, rotation,
                                          translation, camera_model.image_shape)
    visible = np.zeros(len(points), dtype=bool)
    indices = selected[projection['supported']]
    visible[indices] = camera_ground_unoccluded(grid, translation[:2], points[indices])
    unknown = visible & (~free & ~grid.occupied).ravel()
    count = int(np.count_nonzero(visible))
    unknown_count = int(np.count_nonzero(unknown))
    return dict(available=True, reason='CAMERA_GROUND_SUPPORT',
                gain=float(unknown_count / max(1, count)),
                unknown_cells=world_cells_from_mask(grid, unknown),
                visible_cells=world_cells_from_mask(grid, visible) if include_visible else set(),
                covered_count=count, projected_count=int(len(indices)),
                selected_count=selected_count, selected_unknown_count=selected_unknown_count,
                roi_unknown_coverage=float(unknown_count / max(1, selected_unknown_count)))


def observation_task_roi(grid, free, point, target, length=1.):
    """查询目标方向下一段机身扫掠包络缺少的地面证据；未知格绝不因此成为路径边。"""
    point, target = np.asarray(point, dtype=float), np.asarray(target, dtype=float)
    if (point.shape != (2,) or target.shape != (2,) or not np.isfinite(point).all()
            or not np.isfinite(target).all() or not math.isfinite(length) or length <= 0.):
        return set()
    direction = target - point
    distance = float(np.linalg.norm(direction))
    if distance <= 1e-9:
        return set()
    direction /= distance
    length = min(length, distance)
    end = point + length * direction
    # 固定朝向矩形的前移扫掠仍为矩形，查询范围与实际长宽相同，不再使用圆形胶囊。
    heading = math.atan2(direction[1],direction[0])
    center = (point+end)/2
    from .footprint import RectangleFootprint
    vertices = RectangleFootprint(grid.footprint.length+length,grid.footprint.width).vertices(*center,heading)
    rows_cols = polygon_cells(vertices,np.zeros(2),grid.resolution)
    cells = rows_cols[:,::-1]
    local = cells - np.rint(grid.origin / grid.resolution).astype(int)
    inside = np.all((local >= 0) & (local < grid.size), axis=1)
    missing = np.ones(len(cells), dtype=bool)
    indices = np.flatnonzero(inside)
    # 已知障碍回答了该格不可走，不能再把它作为待揭示的“自由空间收益”。
    missing[indices] = ~free[local[indices, 1], local[indices, 0]] & ~grid.occupied[local[indices, 1], local[indices, 0]]
    # 窗口外仍是缺失证据，不裁掉分母；ROI 只是询问范围，执行仍限于原 allowed。
    return set(map(tuple, cells[missing].tolist()))


def observation_attempted(tried, point, heading, resolution=.1):
    """按已执行的区域与视向去重；旧三项记录仍按原单点半径识别。"""
    cell = tuple(math.floor(value/resolution) for value in point)
    for attempt in tried:
        difference = (heading-attempt[2]+math.pi) % (2*math.pi)-math.pi
        if abs(difference) >= .35:
            continue
        if len(attempt) >= 4:
            # 区域准入允许在参考点之前停下，去重也必须覆盖同一任务的完整区域。
            # 只比较原区域的世界格，不把附近其他位置或不同朝向一并封锁。
            if any(tuple(math.floor(value/resolution) for value in position) == cell
                   for position in attempt[3]):
                return True
        elif math.dist(point,attempt[:2]) < .2:
            return True
    return False


def world_cells(grid, rows, cols):
    """将窗口内行列转成固定世界格编号，滚动窗口移动不改变同一地面的身份。"""
    mask = np.zeros(grid.size*grid.size,dtype=bool)
    mask[rows*grid.size+cols] = True
    return world_cells_from_mask(grid,mask)


def world_cells_from_mask(grid, mask):
    """仅在对外返回所选视野时组装世界格，内部评分不创建 Python 坐标集合。"""
    offset = np.rint(grid.origin/grid.resolution).astype(int)
    flat = np.flatnonzero(mask)
    return set(zip((flat%grid.size+offset[0]).tolist(),(flat//grid.size+offset[1]).tolist()))


def local_cell_mask(grid, selected_cells):
    """将相关世界格映射至当前窗口；窗口外格不出现于mask，但仍保留在上层重叠分母。"""
    mask = np.zeros(grid.size*grid.size,dtype=bool)
    selected = np.asarray(list(selected_cells),dtype=int)
    if not len(selected):
        return mask
    local = selected-np.rint(grid.origin/grid.resolution).astype(int)
    inside = (local[:,0] >= 0) & (local[:,0] < grid.size) & (local[:,1] >= 0) & (local[:,1] < grid.size)
    local = local[inside]
    mask[local[:,1]*grid.size+local[:,0]] = True
    return mask


@lru_cache(maxsize=4)
def ray_samples(resolution):
    """缓存半格间距的前视覆盖，最远处横向间距和径向步长都不超过半格。"""
    step = resolution*.5
    count = math.ceil(1.36*3.1/step)+1
    angles = np.linspace(-.68,.68,count)
    ranges = np.linspace(0.,3.1,math.ceil(3.1/step)+1)
    return angles,ranges


def observation_masks(grid, free, points, headings, include_visible=True):
    """同时覆盖少量视野并用格mask去重，保留密射线逐线遮挡语义，避免集合与排序。"""
    points = np.asarray(points,dtype=float).reshape(-1,2)
    headings = np.asarray(headings,dtype=float).reshape(-1)
    count = len(points)
    offsets,ranges = ray_samples(grid.resolution)
    angles = headings[:,None]+offsets[None,:]
    # 分别计算行列，减少临时 xy/indices 三维数组；连续位置与朝向不量化、不缓存旧地图结果。
    col = np.floor((points[:,0,None,None]+np.cos(angles)[:,:,None]*ranges-grid.origin[0])/grid.resolution).astype(int)
    row = np.floor((points[:,1,None,None]+np.sin(angles)[:,:,None]*ranges-grid.origin[1])/grid.resolution).astype(int)
    valid = (col >= 0) & (col < grid.size) & (row >= 0) & (row < grid.size)
    np.clip(col,0,grid.size-1,out=col)
    np.clip(row,0,grid.size-1,out=row)
    # 射线一旦遇到已知障碍或窗口外，后面的未知不能被虚构成可观察收益。
    blocked = grid.occupied[row,col] | ~valid
    # 第一个障碍表面本身可见，但其后方不可见；未知不能凭空作为遮挡物或自由格。
    previous_blocked = np.cumsum(blocked,axis=2)-blocked
    visible = (previous_blocked == 0) & valid
    # 近于 0.7 m 的覆盖只负责发现遮挡；收益仍来自原 0.7~3.1 m 范围。
    eligible = visible & (ranges >= .7-1e-9)
    # 每视野分配一个连续格mask，重复采样写同一 True，替代三次 np.unique 排序。
    flat = row*grid.size+col+np.arange(count)[:,None,None]*(grid.size*grid.size)
    covered = np.zeros((count,grid.size*grid.size),dtype=bool)
    covered.ravel()[flat[eligible]] = True
    unknown = covered & (~free & ~grid.occupied).ravel()[None,:]
    covered_count = np.count_nonzero(covered,axis=1)
    gains = np.count_nonzero(unknown,axis=1)/np.maximum(1,covered_count)
    visible_mask = None
    if include_visible:
        visible_mask = np.zeros_like(covered)
        visible_mask.ravel()[flat[visible]] = True
    return dict(gains=gains,unknown_masks=unknown,visible_masks=visible_mask,
                covered_counts=covered_count,ray_count=len(offsets),sample_count=len(offsets)*len(ranges))


def observation_gains(grid, free, points, headings):
    """分批仅计算候选数值收益，限制临时内存；物理覆盖与最终观察验证使用同一核心。"""
    points,headings = np.asarray(points),np.asarray(headings)
    gains = np.empty(len(points))
    # 十六视野批次在 0.1 m 地图约 8.7 万采样，不一次构造全部 128 视野的临时数组。
    for begin in range(0,len(points),16):
        end = begin+16
        gains[begin:end] = observation_masks(grid,free,points[begin:end],headings[begin:end],False)['gains']
    return gains


def observation_view(grid, free, point, heading):
    """按地图分辨率覆盖原前视扇区，返回唯一世界格上的未知比例及可见格集合。"""
    view = observation_masks(grid,free,[point],[heading])
    return dict(gain=float(view['gains'][0]),
                unknown_cells=world_cells_from_mask(grid,view['unknown_masks'][0]),
                visible_cells=world_cells_from_mask(grid,view['visible_masks'][0]),
                covered_count=int(view['covered_counts'][0]),ray_count=view['ray_count'],sample_count=view['sample_count'])


def unknown_world_cells(grid, free, point, heading):
    """获取当前视向预测待揭示的世界格；它们只是预测，不能计作已经观测的信息。"""
    return observation_view(grid,free,point,heading)['unknown_cells']


def check_observation_position(grid, point, heading, expected_cells, now,
                               camera_model=None, camera_observation=None, initial_yaw=None):
    """验证实际连续位置的净空、预测收益及原前沿覆盖；搜索连通性仍由上层验证。"""
    free,_,clearance = grid.layers(now)
    expected = set(map(tuple,expected_cells))
    camera_observation = (getattr(grid, 'camera_observation', False) if camera_observation is None
                          else bool(camera_observation))
    if camera_observation:
        model = getattr(grid, 'camera_model', None) if camera_model is None else camera_model
        view = camera_ground_view(grid, free, point, heading, model, selected_cells=expected)
        gain = view['roi_unknown_coverage']
    else:
        view = observation_view(grid,free,point,heading)
        gain = view['gain']
    overlap = view['visible_cells'] & expected
    ratio = len(overlap)/max(1,len(expected))
    cell = grid.cell(*point)
    distance = 0. if cell is None else float(clearance[cell])
    # 净空距离只供解释；几何准入由实际矩形及完整转身扫掠判断。
    footprint_clear = cell is not None and grid.motion_clear(
        free,[*point,heading if initial_yaw is None else initial_yaw],[*point,heading],OBSERVATION_DRIFT_MARGIN)
    if cell is None:
        reason = 'OBSERVATION_OUTSIDE_MAP'
    elif not footprint_clear:
        reason = 'OBSERVATION_NO_DRIFT_SPACE'
    elif camera_observation and not view['available']:
        reason = 'CAMERA_GEOMETRY_UNAVAILABLE'
    elif ratio < OBSERVATION_MIN_OVERLAP:
        reason = 'OBSERVATION_FRONTIER_MISMATCH'
    elif gain < OBSERVATION_MIN_GAIN:
        reason = 'OBSERVATION_LOW_GAIN'
    else:
        reason = 'OBSERVATION_POSITION_VALID'
    # 使用可见格而非仍未知格计算重叠：移动中已揭示的相关格不应因变已知而丢失身份。
    return dict(valid=reason == 'OBSERVATION_POSITION_VALID',footprint_clear=footprint_clear,
                geometry_valid=reason in ('OBSERVATION_POSITION_VALID','OBSERVATION_LOW_GAIN'),reason=reason,
                gain=gain,overlap_ratio=ratio,overlap_count=len(overlap),
                geometry_mode='camera' if camera_observation else 'cone',
                projected_count=view.get('projected_count'),selected_count=view.get('selected_count'),
                clearance=distance,unknown_cells=view['unknown_cells'],visible_cells=view['visible_cells'])


def observation_cell_state(grid, selected_cells, now):
    """统计指定世界格的真实已知、自由、可通行和障碍状态，不把预测收益当测量。"""
    result = dict(known=set(),free=set(),allowed=set(),occupied=set())
    selected = np.asarray(sorted(set(map(tuple,selected_cells))),dtype=int)
    if not len(selected):
        return result
    offset = np.rint(grid.origin/grid.resolution).astype(int)
    local = selected-offset
    inside = np.all((local >= 0) & (local < grid.size),axis=1)
    selected,local = selected[inside],local[inside]
    if not len(selected):
        return result
    cols,rows = local[:,0],local[:,1]
    free,allowed,_ = grid.layers(now)
    occupied = grid.occupied[rows,cols]
    # 障碍表面也是真实新信息；已知不意味着自由，更不意味着完整包络可通行。
    known = (np.isfinite(grid.seen[rows,cols]) & (grid.seen[rows,cols] <= now)) | occupied
    masks = dict(known=known,free=free[rows,cols],allowed=allowed[rows,cols],occupied=occupied)
    for name,mask in masks.items():
        result[name] = set(map(tuple,selected[mask].tolist()))
    return result
