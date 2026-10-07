"""仅从前视深度更新的平地滚动地图；未知空间不等于自由空间。"""
import math
import cv2
import numpy as np
from .footprint import RectangleFootprint, lattice_layers, pose_clear, motion_clear, route_clear, polygon_cells
from .navigation_config import ROBOT_LENGTH, ROBOT_WIDTH


class RollingMap:
    """保存观测时间与持久障碍，提供矩形机身各朝向的可通行姿态。"""
    def __init__(self, size=121, resolution=0.10, free_ttl=30.0, radius=None, static_history=False, footprint=None):
        """使用用户指定的中心矩形；腿部运动超出该长宽时需重新测量包络。"""
        self.size, self.resolution = size, resolution
        self.free_ttl = free_ttl
        # radius 仅兼容旧测试构造的小方形，不会在运行中恢复圆形包络。
        self.footprint = footprint or (RectangleFootprint(max(1e-8,2*radius),max(1e-8,2*radius)) if radius is not None else
                                      RectangleFootprint(ROBOT_LENGTH,ROBOT_WIDTH))
        self.radius = min(self.footprint.length,self.footprint.width)/2
        self._geometry_cache = None
        self._window_cache = None
        self.static_history = static_history
        cv2.setNumThreads(1)
        self.origin = np.array([-size // 2, -size // 2], dtype=float) * resolution
        self.seen = np.full((size, size), -np.inf)
        self.occupied = np.zeros((size, size), dtype=bool)
        self.last_depth = None
        self.frames = 0
        self.confirmed = False
        self.version = 0
        # 窗口平移只改变数组布局；观测/人工确认才改变世界坐标中的证据内容。
        self.evidence_version = 0
        # 标定只用于预测取景几何；不会把预测支持写成已观测自由格。
        self.camera_observation = False
        self.camera_model = None

    def centers(self):
        """返回各栅格中心的 odom 坐标，数组按行 y、列 x 排列。"""
        y, x = np.indices(self.seen.shape)
        return self.origin[0] + (x + 0.5) * self.resolution, self.origin[1] + (y + 0.5) * self.resolution

    def recenter(self, x, y):
        """按整格移动窗口，保留重叠观测，新增区域始终为未知。"""
        origin = (np.floor(np.array([x, y]) / self.resolution) - self.size // 2) * self.resolution
        delta = np.rint((origin - self.origin) / self.resolution).astype(int)
        if not np.any(delta):
            return
        seen, occupied = np.full_like(self.seen, -np.inf), np.zeros_like(self.occupied)
        if np.max(np.abs(delta)) < self.size:
            dx, dy = delta
            old_x, old_y = slice(max(dx, 0), min(self.size + dx, self.size)), slice(max(dy, 0), min(self.size + dy, self.size))
            new_x, new_y = slice(max(-dx, 0), min(self.size - dx, self.size)), slice(max(-dy, 0), min(self.size - dy, self.size))
            seen[new_y, new_x], occupied[new_y, new_x] = self.seen[old_y, old_x], self.occupied[old_y, old_x]
        self.origin, self.seen, self.occupied = origin, seen, occupied
        self.version += 1

    def confirm_start(self, x, y, now):
        """接受操作者对半径 1.2 m 起始净空的单次确认；不能清除已观测障碍。"""
        if self.confirmed:
            return False
        self.recenter(x, y)
        gx, gy = self.centers()
        area = (gx - x) ** 2 + (gy - y) ** 2 <= 1.2 ** 2
        if np.any(self.occupied & area):
            return False
        self.seen[area] = now
        self.confirmed = True
        self.version += 1
        self.evidence_version += 1
        return True

    def integrate(self, depth, intrinsic, rotation, translation, stamp):
        """用采集时刻 optical→odom 外参投影深度，不访问场景几何或目标真值。"""
        if self.last_depth is not None and stamp <= self.last_depth:
            return False
        if depth.ndim != 2 or not np.isfinite(intrinsic).all() or not np.isfinite(rotation).all() or not np.isfinite(translation).all() or not math.isfinite(stamp):
            return False
        fx, fy, cx, cy = intrinsic
        if fx <= 0 or fy <= 0:
            return False
        height, width = depth.shape
        valid = np.isfinite(depth) & (depth >= 0.25) & (depth <= 5.0)
        if int(valid.sum()) < max(20, depth.size // 100):
            # 图像仍在到达但已经没有有效测量，同样不能刷新传感器健康时间。
            return False
        # 无效深度不能清空地图；局部最小值避免边缘像素插值把墙角当作通道。
        conservative = cv2.erode(np.where(valid, depth, 0).astype(np.float32), np.ones((3, 3), np.uint8))
        gx, gy = self.centers()
        xy = np.column_stack((gx.ravel(), gy.ravel()))
        # 平地假设 z=0；检查机器人高度带的多个截面，不能用一条贴地射线清除高处障碍。
        heights = np.arange(0.08, 0.75, 0.08)
        points = np.empty((len(xy), len(heights), 3))
        points[:, :, :2], points[:, :, 2] = xy[:, None, :], heights
        camera = (points - translation) @ rotation
        z = camera[:, :, 2]
        safe_z = np.maximum(z, 1e-6)
        u = np.rint(fx * camera[:, :, 0] / safe_z + cx).astype(int)
        v = np.rint(fy * camera[:, :, 1] / safe_z + cy).astype(int)
        inside = (z >= 0.25) & (z <= 4.8) & (u >= 1) & (u < width - 1) & (v >= 1) & (v < height - 1)
        sampled = conservative[np.clip(v, 0, height - 1), np.clip(u, 0, width - 1)]
        clear = np.all(inside & (sampled > z + self.resolution), axis=1).reshape(self.seen.shape)
        self.seen[clear], self.occupied[clear] = stamp, False
        # 平地工作域允许用实际可见地面建立支撑区域；天空无返回不要求变成自由体素。
        # 已存在的高处障碍仍只能由上面的高度带观测清除，地面可见不能清掉桌面等障碍。
        ground = np.column_stack((xy, np.zeros(len(xy))))
        optical = (ground - translation) @ rotation
        gz = optical[:, 2]
        gu = np.rint(fx * optical[:, 0] / np.maximum(gz, 1e-6) + cx).astype(int)
        gv = np.rint(fy * optical[:, 1] / np.maximum(gz, 1e-6) + cy).astype(int)
        ground_inside = (gz > 0.3) & (gz < 4.8) & (gu >= 1) & (gu < width - 1) & (gv >= 1) & (gv < height - 1)
        # 地面相邻像素不是同一深度：先逐像素计算其射线与z=0平面的交点，
        # 再要求3×3邻域全部匹配各自的地面深度。对最近深度直接做差会误拒绝远处平地。
        ground_seen = ground_inside.copy()
        for du in (-1,0,1):
            for dv in (-1,0,1):
                pu,pv = gu+du,gv+dv
                ray = np.column_stack(((pu-cx)/fx,(pv-cy)/fy,np.ones(len(pu))))
                world_ray = ray@rotation.T
                downward = world_ray[:,2] < -1e-6
                expected = np.divide(-translation[2],world_ray[:,2],out=np.full(len(pu),np.inf),where=downward)
                measured = depth[np.clip(pv,0,height-1),np.clip(pu,0,width-1)]
                ground_seen &= downward & np.isfinite(measured) & (expected>.3) & (expected<4.8)
                ground_seen &= abs(measured-expected) < .04+.015*expected
        ground_seen = ground_seen.reshape(self.seen.shape)
        self.seen[ground_seen & ~self.occupied] = stamp
        # 表面点优先于自由空间；只有新的完整自由观测才能清掉旧障碍，超时不会清障碍。
        rows, cols = np.nonzero(valid)
        distances = depth[rows, cols]
        cloud = np.column_stack(((cols - cx) * distances / fx, (rows - cy) * distances / fy, distances))
        world = cloud @ rotation.T + translation
        hit = (world[:, 2] >= 0.06) & (world[:, 2] <= 0.80)
        indices = np.floor((world[hit, :2] - self.origin) / self.resolution).astype(int)
        indices = indices[np.all((indices >= 0) & (indices < self.size), axis=1)]
        if len(indices):
            self.occupied[indices[:, 1], indices[:, 0]] = True
            # 障碍命中也是实际观测：记录采集时间，观察任务才能区分新看到的墙与历史障碍。
            # free 始终排除 occupied，因此更新时间不会把障碍变成可行驶空间。
            self.seen[indices[:, 1], indices[:, 0]] = stamp
        self.last_depth, self.frames = stamp, self.frames + 1
        self.version += 1
        self.evidence_version += 1
        return True

    def layers(self, now):
        """按静态历史或新鲜观测模式取净空，再对障碍、未知与窗口边缘膨胀。"""
        # 静态场景保留历史几何；观测年龄仍单独显示，不能由此推断盲区没有动态侵入。
        known = np.isfinite(self.seen) & (self.seen <= now)
        free = known & ~self.occupied
        if not self.static_history:
            free &= now - self.seen <= self.free_ttl
        # 本拍自由证据是只读快照，缓存完整区域查询不会漏掉后续地图更新。
        free.flags.writeable = False
        padded = np.pad(free.astype(np.uint8), 1)
        clearance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1] * self.resolution
        # 再扣半格对角线，抵消点中心距离与栅格实际边界的差异。
        # 任一朝向能容纳矩形的二维并集；不能用它单独保证当前朝向或转身安全。
        allowed = np.any(self.orientation_layers(free)[0],axis=0)
        return free, allowed, clearance

    def window_free(self, free, low_x, low_y, high_x, high_y):
        """判定轴对齐框内每格都自由；仅作矩形扫掠的充分条件，失败仍做精确校验。"""
        if min(low_x,low_y)<0 or max(high_x,high_y)>=self.size:
            return False
        if free.flags.writeable:
            # 单元测试和外部可写掩码不能缓存，否则原地改格后会使用旧结果。
            return all(bool(free[row,col]) for row in range(low_y,high_y+1)
                       for col in range(low_x,high_x+1))
        if self._window_cache is None or self._window_cache[0] is not free:
            blocked = (~free).astype(np.int32)
            prefix = np.pad(blocked.cumsum(0).cumsum(1),((1,0),(1,0)))
            self._window_cache = (free,prefix)
        prefix = self._window_cache[1]
        count = (int(prefix[high_y+1,high_x+1])-int(prefix[low_y,high_x+1])
                 -int(prefix[high_y+1,low_x])+int(prefix[low_y,low_x]))
        return count == 0

    def orientation_layers(self, free, margin=0.):
        """缓存当前自由证据的八朝向矩形姿态及完整运动边；缓存不作为地图证据。"""
        key = (hash(free.tobytes()),float(margin),self.resolution,self.footprint)
        if self._geometry_cache is None or self._geometry_cache[0] != key:
            self._geometry_cache = (key,lattice_layers(self,free,margin))
        return self._geometry_cache[1]

    def pose_clear(self, free, x, y, yaw, margin=0.):
        """核验当前连续位置的矩形包络，朝向不可省略。"""
        return pose_clear(self,free,x,y,yaw,margin)

    def motion_clear(self, free, a, b, margin=0.):
        """核验平移或旋转全过程的矩形扫掠。"""
        return motion_clear(self,free,a,b,margin)

    def route_clear(self, free, route, initial_yaw, final_yaw=None, margin=0.):
        """核验路线及转角的姿态可达性，不能只验证中心线。"""
        return route_clear(self,free,route,initial_yaw,final_yaw,margin)

    def footprint_status(self, x, y, now, yaw=0.):
        """区分窗口边界、障碍、从未观察和过期，记录具体阻塞位置而非笼统报未知。"""
        cell = self.cell(x, y)
        free, allowed, clearance = self.layers(now)
        if cell is None:
            return dict(code='MAP_WINDOW', clearance=0.0)
        vertices = self.footprint.vertices(x,y,yaw)
        touched = polygon_cells(vertices,self.origin,self.resolution,self.size)
        if touched is None:
            return dict(code='MAP_WINDOW',clearance=float(clearance[cell]),vertices=vertices.tolist())
        if np.all(free[touched[:,0],touched[:,1]]):
            return dict(code='CLEAR', clearance=float(clearance[cell]))
        area = np.zeros_like(free)
        area[touched[:,0],touched[:,1]] = True
        for code, mask in [('FOOTPRINT_OCCUPIED', self.occupied),
                           ('FOOTPRINT_UNKNOWN', ~np.isfinite(self.seen)),
                           ('FOOTPRINT_EXPIRED', ~free & ~self.occupied)]:
            bad = np.argwhere(area & mask)
            if len(bad):
                index = tuple(bad[0])
                return dict(code=code, clearance=float(clearance[cell]), cell=self.point(index),
                            age=None if not np.isfinite(self.seen[index]) else float(now-self.seen[index]))
        return dict(code='MAP_WINDOW', clearance=float(clearance[cell]))

    def cell(self, x, y):
        """世界坐标转换为行列；窗口外返回 None。"""
        col, row = np.floor((np.array([x, y]) - self.origin) / self.resolution).astype(int)
        return (int(row), int(col)) if 0 <= row < self.size and 0 <= col < self.size else None

    def point(self, cell):
        """将行列转换为栅格中心坐标。"""
        row, col = cell
        return (self.origin + (np.array([col, row]) + 0.5) * self.resolution).tolist()

    def permitted(self, allowed, x, y):
        """检查单个机器人中心位置是否有完整的可信净空。"""
        cell = self.cell(x, y)
        return cell is not None and bool(allowed[cell])

    def segment_cells(self, a, b):
        """返回线段穿过或接触的全部 (行,列)；越界或非有限输入返回 None。

        包含端点、沿格线的两侧和格角的四格。零长度仍返回接触格，
        因此调用方不能把空列表误当成已经验证安全的线段。
        """
        try:
            # 常见输入直接读四个标量；停车积分每拍有大量短段，避免逐段创建数组。
            if (len(a) != 2 or len(b) != 2 or isinstance(a, (str, bytes, dict))
                    or isinstance(b, (str, bytes, dict))
                    or getattr(a, 'shape', (2,)) != (2,) or getattr(b, 'shape', (2,)) != (2,)):
                return None
            # NumPy 长度一数组也能转 float，但嵌套数组不是二维端点，不能误接收。
            if (getattr(a[0], 'ndim', 0) != 0 or getattr(a[1], 'ndim', 0) != 0
                    or getattr(b[0], 'ndim', 0) != 0 or getattr(b[1], 'ndim', 0) != 0):
                return None
            ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
        except (TypeError, ValueError, IndexError):
            return None
        if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(bx) and math.isfinite(by)):
            return None
        ox, oy = float(self.origin[0]), float(self.origin[1])
        x0, y0 = (ax-ox)/self.resolution, (ay-oy)/self.resolution
        x1, y1 = (bx-ox)/self.resolution, (by-oy)/self.resolution
        if min(x0, y0, x1, y1) < 0. or max(x0, y0, x1, y1) > self.size:
            return None
        col0, row0, col1, row1 = math.floor(x0), math.floor(y0), math.floor(x1), math.floor(y1)
        # 控制积分的大多数短段完全位于单个格内部；凸格内直线不会触碰其他格。
        # 边界点仍走完整覆盖，不能把擦角与零长边界检查优化掉。
        if (col0 == col1 and row0 == row1 and col0 < self.size and row0 < self.size
                and 1e-9 < x0-col0 < 1.-1e-9 and 1e-9 < y0-row0 < 1.-1e-9
                and 1e-9 < x1-col1 < 1.-1e-9 and 1e-9 < y1-row1 < 1.-1e-9):
            return [(row0, col0)]
        dx, dy = x1-x0, y1-y0
        if max(abs(dx), abs(dy)) <= 8.:
            # 仍按旧算法求交点及相邻中点，不用 DDA 合并近似相等的交点时间。
            # 两轴交点时间即使只差一个浮点位，也必须保留，以免漏掉极短擦角格。
            times = {0., 1.}
            for start, end, delta_axis in ((x0, x1, dx), (y0, y1, dy)):
                if delta_axis == 0.:
                    continue
                for border in range(math.ceil(min(start, end)), math.floor(max(start, end))+1):
                    crossing = (border-start)/delta_axis
                    if 0. <= crossing <= 1.:
                        times.add(crossing)
            times = sorted(times)
            checks = times + [(left+right)*.5 for left, right in zip(times, times[1:])]
            flat = set()
            for time in checks:
                x, y = x0+time*dx, y0+time*dy
                rounded_x, rounded_y = round(x), round(y)
                touches_x, touches_y = abs(x-rounded_x) <= 1e-9, abs(y-rounded_y) <= 1e-9
                upper_x = rounded_x if touches_x else math.floor(x)
                upper_y = rounded_y if touches_y else math.floor(y)
                lower_x, lower_y = upper_x-touches_x, upper_y-touches_y
                # 沿格线检查两侧，格角检查四格；窗口外接触直接拒绝，不裁剪。
                if lower_x < 0 or lower_y < 0 or upper_x >= self.size or upper_y >= self.size:
                    return None
                flat.add(upper_y*self.size+upper_x)
                if touches_x:
                    flat.add(upper_y*self.size+lower_x)
                if touches_y:
                    flat.add(lower_y*self.size+upper_x)
                    if touches_x:
                        flat.add(lower_y*self.size+lower_x)
            return [(cell//self.size, cell%self.size) for cell in sorted(flat)]
        # 长段继续用向量计算；切换阈值只影响算力，不改变接触格与安全阈值。
        points = np.array(((x0, y0), (x1, y1)))
        delta = points[1]-points[0]
        times = [np.array([0., 1.])]
        for axis in range(2):
            if delta[axis] == 0.:
                continue
            low, high = min(points[:,axis]), max(points[:,axis])
            borders = np.arange(math.ceil(low), math.floor(high)+1, dtype=float)
            crossing = (borders-points[0,axis])/delta[axis]
            times.append(crossing[(crossing >= 0.) & (crossing <= 1.)])
        times = np.unique(np.concatenate(times))
        # 相邻交点之间不会再跨格线：中点覆盖内部，交点覆盖擦角和端点接触。
        checks = np.concatenate((times, (times[:-1]+times[1:])*.5))
        positions = points[0]+checks[:,None]*delta
        rounded = np.rint(positions)
        # 只吸收格坐标计算的浮点误差；边界接触同时检查两侧，不向安全侧取整。
        touches = np.abs(positions-rounded) <= 1e-9
        upper = np.where(touches, rounded, np.floor(positions)).astype(int)
        lower = upper-touches.astype(int)
        columns = np.concatenate((upper[:,0], lower[:,0], upper[:,0], lower[:,0]))
        rows = np.concatenate((upper[:,1], upper[:,1], lower[:,1], lower[:,1]))
        # 触碰窗口外同样未知，不能只裁掉越界格之后宣告安全。
        if np.any(columns < 0) or np.any(columns >= self.size) or np.any(rows < 0) or np.any(rows >= self.size):
            return None
        flat = np.unique(rows*self.size+columns)
        return list(zip((flat//self.size).tolist(), (flat%self.size).tolist()))

    def segment_clear(self, allowed, a, b):
        """逐格校验线段的完整覆盖，供搜索简化与执行校验共用，禁止漏格和斜穿角。"""
        cells = self.segment_cells(a, b)
        if cells is None:
            return False
        # 短段只有少量格，逐格读取避免数组分配，并在第一个禁行格及时结束。
        if len(cells) <= 16:
            return all(allowed[row, col] for row, col in cells)
        indices = np.asarray(cells, dtype=int)
        return bool(np.all(allowed[indices[:,0],indices[:,1]]))

    def display(self, now):
        """输出紧凑地图：0 未知、1 自由、2 障碍、3 包络可通行、4 历史自由边缘。"""
        free, allowed, _ = self.layers(now)
        grid = np.zeros_like(self.seen, dtype=np.uint8)
        grid[free], grid[self.occupied], grid[allowed] = 1, 2, 3
        grid[free & (now-self.seen > self.free_ttl) & ~allowed] = 4
        return dict(origin=self.origin.tolist(), resolution=self.resolution, size=self.size,
                    cells=''.join(map(str, grid.ravel().tolist())),
                    footprint=dict(shape='rectangle',length=self.footprint.length,width=self.footprint.width),
                    allowed_semantics='any_lattice_heading; actual pose and sweep checked separately',
                    frames=self.frames, confirmed=self.confirmed,
                    observation_mode='camera' if self.camera_observation else 'cone',
                    camera_geometry_available=self.camera_model is not None,
                    camera_geometry=(None if self.camera_model is None else
                                     dict(intrinsic=list(self.camera_model.intrinsic),
                                          image_shape=list(self.camera_model.image_shape),
                                          base_rotation=np.asarray(self.camera_model.base_rotation).tolist(),
                                          base_translation=np.asarray(self.camera_model.base_translation).tolist())),
                    mode='static_history' if self.static_history else 'fresh_only',
                    historical_free_cells=int(np.count_nonzero(free & (now-self.seen > self.free_ttl))),
                    depth_age=None if self.last_depth is None else now - self.last_depth)
