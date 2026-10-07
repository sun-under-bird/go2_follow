"""从有效 UWB 世界轨迹形成弧长跟随意图；不提供自由空间或机器人可执行路径。"""
from dataclasses import dataclass
import math
from .navigation_config import FOLLOW_REGION_HALF_WIDTH, FOLLOW_GOAL_TOLERANCE


@dataclass(frozen=True)
class IntentHint:
    """人的轨迹上的跟随提示；path 是意图弧段，必须另经传感器地图规划与碰撞检查。"""
    follow_point: tuple
    path: tuple
    latest_target: tuple
    predicted_target: tuple
    target_stamp: float
    age: float
    prediction_seconds: float
    observed_length: float
    history_complete: bool
    virtual_stem_used: bool
    source: str


@dataclass(frozen=True)
class IntentGuide:
    """完整有限意图与顺序标签；orders 用于连续投影，不能充当虚拟点的传感器时间戳。"""
    path: tuple
    orders: tuple
    sequence: int
    latest_target: tuple
    predicted_target: tuple
    target_stamp: float
    age: float
    prediction_seconds: float
    observed_length: float
    virtual_stem_available: bool


@dataclass(frozen=True)
class IntentReference:
    """机器人当前进展对应的搜索参考；path 仍是意图，绝非已验证的执行路径。"""
    goal: tuple
    path: tuple
    projection: tuple
    projection_gap: float
    projection_order: float
    remaining_arc: float
    next_corner: tuple
    status: str
    source: str
    sequence: int
    target_stamp: float
    age: float
    prediction_seconds: float
    observed_length: float
    virtual_stem_used: bool
    corner_evidence: tuple = ()

    def diagnostics(self):
        """提供可序列化的意图来源和进展，避免界面把人体轨迹画成机器人的安全路径。"""
        return dict(goal=list(self.goal), intent_path=[list(point) for point in self.path],
                    projection=list(self.projection), projection_gap=self.projection_gap,
                    projection_order=self.projection_order, remaining_arc=self.remaining_arc,
                    next_corner=None if self.next_corner is None else list(self.next_corner),
                    status=self.status, source=self.source, sequence=self.sequence,
                    target_stamp=self.target_stamp, age=self.age, prediction_seconds=self.prediction_seconds,
                    observed_length=self.observed_length, virtual_stem_used=self.virtual_stem_used,
                    corner_evidence=dict(self.corner_evidence),
                    execution_path=False)


class FollowIntent:
    """保留人的有效拐点与折返，避免当前目标加直线外推抹掉已经走过的绕行意图。"""
    def __init__(self, sample_distance=.08, straight_tolerance=.03, corner_angle=.35,
                 max_history_length=12., max_points=256, stale_timeout=.45,
                 max_prediction=.5, max_speed=1.5, stationary_speed=.04):
        """配置空间降采样、共线合并和有限历史；阈值是意图过滤参数，不是速度补偿。"""
        finite = (sample_distance, straight_tolerance, corner_angle, max_history_length,
                  stale_timeout, max_prediction, max_speed, stationary_speed)
        if (not all(math.isfinite(value) for value in finite) or sample_distance <= 0.
                or straight_tolerance < 0. or not 0. < corner_angle < math.pi
                or max_history_length < sample_distance or max_points < 4
                or stale_timeout <= 0. or max_prediction < 0. or max_speed <= 0.
                or stationary_speed < 0.):
            raise ValueError('跟随意图参数必须有限，历史和点数上限须能容纳有效路径')
        self.sample_distance, self.straight_tolerance = sample_distance, straight_tolerance
        self.corner_angle, self.max_history_length = corner_angle, max_history_length
        self.max_points, self.stale_timeout = int(max_points), stale_timeout
        self.max_prediction, self.max_speed = max_prediction, max_speed
        self.stationary_speed = stationary_speed
        self.reset()

    def reset(self):
        """场景或定位坐标系重置时清空历史，允许新的单调时间序列从任意时刻开始。"""
        self._sequence = getattr(self, '_sequence', 0)+1
        self._points = []
        self._latest = None
        self._stamp = None
        self._velocity = (0., 0.)
        self._virtual_start = None
        self._initial_target = None
        self._valid = False

    @staticmethod
    def _point(value):
        """校验二维世界坐标或速度并复制成不可变值，不修改调用方输入。"""
        try:
            if len(value) != 2:
                return None
            point = float(value[0]), float(value[1])
        except (TypeError, ValueError, OverflowError):
            return None
        return point if all(math.isfinite(item) for item in point) else None

    @staticmethod
    def _length(points):
        """计算有顺序轨迹的弧长，来回走同一条线也保留实际经过的路程。"""
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    def _mergeable(self, first, middle, last):
        """仅合并近共线且同向的中间点；明显转角和反向折返点必须保留。"""
        incoming = middle[0]-first[0], middle[1]-first[1]
        outgoing = last[0]-middle[0], last[1]-middle[1]
        before, after = math.hypot(*incoming), math.hypot(*outgoing)
        if before < 1e-9 or after < 1e-9:
            return True
        cosine = (incoming[0]*outgoing[0]+incoming[1]*outgoing[1])/(before*after)
        # 仅看点到直线距离会删除同线的 180° 折返，必须同时限制方向变化。
        if cosine < math.cos(self.corner_angle):
            return False
        segment = last[0]-first[0], last[1]-first[1]
        squared = segment[0]**2+segment[1]**2
        if squared < 1e-12:
            return False
        ratio = ((middle[0]-first[0])*segment[0]+(middle[1]-first[1])*segment[1])/squared
        ratio = max(0., min(1., ratio))
        projected = first[0]+ratio*segment[0], first[1]+ratio*segment[1]
        return math.dist(middle, projected) <= self.straight_tolerance

    def observed_path(self):
        """返回有限实际目标历史及最新端点；端点小移动不积累成大量噪声折线。"""
        return tuple(point for _, point in self._observed_entries())

    def _observed_entries(self):
        """保留几何降采样对应的测量顺序，供投影在裁剪和共线合并后找到同一进展。"""
        entries = list(self._points)
        if self._latest is not None and (not entries or math.dist(entries[-1][1], self._latest) > 1e-9):
            entries.append((self._stamp, self._latest))
            if len(entries) >= 3 and self._mergeable(*(point for _, point in entries[-3:])):
                del entries[-2]
        return entries

    def guide(self, now, prediction_horizon=.3):
        """提供全部有限历史而非最近间距尾段，落后机器人仍可看见未通过的旧拐点。"""
        if (self._stamp is None or not self._valid
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in (now, prediction_horizon)) or prediction_horizon < 0.):
            return None
        age = now-self._stamp
        if not 0. <= age <= self.stale_timeout:
            return None
        entries = self._observed_entries()
        path = [point for _, point in entries]
        orders = [stamp for stamp, _ in entries]
        observed_length = self._length(path)
        has_virtual = self._virtual_start is not None and path[0] == self._initial_target
        if has_virtual:
            # -1 只建立先后顺序，不声称机器人起点是更早的一帧 UWB 测量。
            path.insert(0, self._virtual_start)
            orders.insert(0, orders[0]-1.)
        seconds = min(self.max_prediction, age+prediction_horizon)
        if math.hypot(*self._velocity) <= self.stationary_speed:
            seconds = 0.
        predicted = tuple(self._latest[i]+self._velocity[i]*seconds for i in range(2))
        if math.dist(path[-1], predicted) > 1e-9:
            path.append(predicted)
            # 预测端点的标签同样只用于排序；真实新鲜度始终取 target_stamp。
            orders.append(self._stamp+seconds)
        return IntentGuide(tuple(path), tuple(orders), self._sequence, self._latest, predicted,
                           self._stamp, age, seconds, observed_length, has_virtual)

    def _trim_history(self):
        """按弧长截掉过旧的头部，再限点数；不删除尾部仍有用的拐点来凑预算。"""
        while len(self._points) > 1:
            overflow = self._length(self.observed_path())-self.max_history_length
            if overflow <= 1e-9:
                break
            first_stamp, first = self._points[0]
            next_stamp, second = self._points[1]
            segment = math.dist(first, second)
            self._virtual_start = None
            if segment <= overflow+1e-9:
                del self._points[0]
            else:
                # 截断点仅是已记录意图线段的插值，不产生地图自由证据。
                ratio = overflow/segment
                point = tuple(first[i]+ratio*(second[i]-first[i]) for i in range(2))
                self._points[0] = first_stamp+ratio*(next_stamp-first_stamp), point
                break
        while len(self._points) > self.max_points-1:
            del self._points[0]
            self._virtual_start = None

    def update(self, target, stamp, valid=True, robot_position=None, velocity=None):
        """只接入有效且时间递增的世界目标；长失联后重建，不连起未观察的丢失轨迹。"""
        if not valid:
            self._valid = False
            return False
        point = self._point(target)
        supplied_velocity = None if velocity is None else self._point(velocity)
        if (point is None or not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                or (velocity is not None and supplied_velocity is None)):
            self._valid = False
            return False
        if self._stamp is not None and stamp <= self._stamp:
            # 旧包只拒收，不能覆盖最新时间/轨迹，也不替代调用方的真实有效性判定。
            return False
        if self._stamp is not None and stamp-self._stamp > self.stale_timeout:
            self.reset()
        if self._stamp is None:
            self._points = [(float(stamp), point)]
            self._initial_target = point
            start = self._point(robot_position)
            self._virtual_start = start if start is not None and math.dist(start, point) > 1e-9 else None
            self._velocity = supplied_velocity or (0., 0.)
        else:
            dt = stamp-self._stamp
            if supplied_velocity is None:
                measured = tuple((point[i]-self._latest[i])/dt for i in range(2))
                alpha = dt/(.3+dt)
                self._velocity = tuple(self._velocity[i]+alpha*(measured[i]-self._velocity[i]) for i in range(2))
            else:
                self._velocity = supplied_velocity
            if math.dist(point, self._points[-1][1]) >= self.sample_distance:
                self._points.append((float(stamp), point))
                while len(self._points) >= 3 and self._mergeable(*(item[1] for item in self._points[-3:])):
                    del self._points[-2]
        speed = math.hypot(*self._velocity)
        if speed <= self.stationary_speed:
            self._velocity = (0., 0.)
        elif speed > self.max_speed:
            self._velocity = tuple(value*self.max_speed/speed for value in self._velocity)
        self._latest, self._stamp, self._valid = point, float(stamp), True
        self._trim_history()
        return True

    def hint(self, now, spacing, prediction_horizon=.3):
        """沿人的有限历史加短时速度预测后退 spacing；失效或过期直接返回 None。"""
        if (self._stamp is None or not self._valid
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in (now, spacing, prediction_horizon))
                or spacing < 0. or prediction_horizon < 0.):
            return None
        age = now-self._stamp
        if not 0. <= age <= self.stale_timeout:
            return None
        observed = list(self.observed_path())
        observed_length = self._length(observed)
        # 初始轨迹太短时只用首次机器人位置补意图，不把机器人到人的直线声明为已观测通道。
        has_virtual = (self._virtual_start is not None and observed[0] == self._initial_target)
        path = [self._virtual_start]+observed if has_virtual else observed
        seconds = min(self.max_prediction, age+prediction_horizon)
        if math.hypot(*self._velocity) <= self.stationary_speed:
            seconds = 0.
        predicted = tuple(self._latest[i]+self._velocity[i]*seconds for i in range(2))
        has_prediction = math.dist(path[-1], predicted) > 1e-9
        if has_prediction:
            path.append(predicted)
        remaining = spacing
        point, segment_index, tail = path[0], 0, list(path)
        for index in range(len(path)-2, -1, -1):
            length = math.dist(path[index], path[index+1])
            if remaining <= length:
                ratio = 0. if length < 1e-9 else remaining/length
                point = tuple(path[index+1][axis]+ratio*(path[index][axis]-path[index+1][axis]) for axis in range(2))
                segment_index, tail = index, [point]+path[index+1:]
                break
            remaining -= length
        virtual_used = has_virtual and segment_index == 0 and math.dist(point, observed[0]) > 1e-9
        source = ('robot_virtual_stem' if virtual_used else 'short_velocity_prediction'
                  if has_prediction and segment_index == len(path)-2 else 'observed_target_trail')
        return IntentHint(point, tuple(tail), self._latest, predicted, self._stamp, age, seconds,
                          observed_length, observed_length >= spacing, virtual_used, source)

    def diagnostics(self):
        """展示意图来源和有限历史，不提供任何 free、allowed 或碰撞结论。"""
        points = self.observed_path()
        return dict(valid=self._valid, target_stamp=self._stamp, point_count=len(points),
                    observed_length=self._length(points), max_history_length=self.max_history_length,
                    max_points=self.max_points, virtual_stem_available=self._virtual_start is not None)


class TrailReferenceTracker:
    """按机器人实走进展读取有顺序的人轨迹，避免在自交或折返处跳到较晚的近点。"""
    def __init__(self, lookahead=2.4, corner_lookahead=.65, projection_budget=.8, corner_angle=.35,
                 corner_support=.30):
        """限定滚动参考与转角方向的空间支持；所有参数仅过滤意图，不改变地图边界。"""
        values = lookahead, corner_lookahead, projection_budget, corner_angle, corner_support
        if (not all(math.isfinite(value) and value > 0. for value in values)
                or corner_angle >= math.pi):
            raise ValueError('轨迹参考参数必须为有限正值，转角阈值须小于 π')
        self.lookahead, self.corner_lookahead = lookahead, corner_lookahead
        self.projection_budget, self.corner_angle = projection_budget, corner_angle
        self.corner_support = corner_support
        self.reset()

    def reset(self):
        """清掉上一个意图序列的进展锚点，不能把不同定位或失联区间连在一起。"""
        self._sequence, self._order, self._robot = None, None, None
        self._budget = self.projection_budget
        self._history_lost = False

    @staticmethod
    def cumulative(path):
        """返回各顶点的累计弧长，重复走过同一点仍有不同的顺序位置。"""
        lengths = [0.]
        for first, second in zip(path, path[1:]):
            lengths.append(lengths[-1]+math.dist(first, second))
        return lengths

    @staticmethod
    def at_arc(path, orders, cumulative, arc):
        """按弧长插值位置和顺序标签，标签仅用于定位已记录意图片段。"""
        if arc <= 0. or len(path) == 1:
            return path[0], orders[0]
        for index in range(len(path)-1):
            if arc <= cumulative[index+1]:
                length = cumulative[index+1]-cumulative[index]
                ratio = 0. if length <= 1e-12 else (arc-cumulative[index])/length
                point = tuple(path[index][axis]+ratio*(path[index+1][axis]-path[index][axis]) for axis in range(2))
                order = orders[index]+ratio*(orders[index+1]-orders[index])
                return point, order
        return path[-1], orders[-1]

    @staticmethod
    def arc_at_order(orders, cumulative, order):
        """用顺序锚点恢复旧进展；历史共线合并不会因数组下标变化而跳到另一圈。"""
        if order <= orders[0]:
            return 0.
        for index in range(len(orders)-1):
            if order <= orders[index+1]:
                difference = orders[index+1]-orders[index]
                ratio = 0. if difference <= 1e-12 else (order-orders[index])/difference
                return cumulative[index]+ratio*(cumulative[index+1]-cumulative[index])
        return cumulative[-1]

    @staticmethod
    def nearest_arc(path, cumulative, robot, lower, upper):
        """只在连续弧长窗口找最近投影，距离相同选较早的一段，禁止自交时跳过旧转角。"""
        if len(path) == 1:
            return 0.
        best_distance, best_arc = math.inf, lower
        for index, (first, second) in enumerate(zip(path, path[1:])):
            length = cumulative[index+1]-cumulative[index]
            begin, end = max(lower, cumulative[index]), min(upper, cumulative[index+1])
            if end < begin-1e-12 or length <= 1e-12:
                continue
            segment = tuple(second[axis]-first[axis] for axis in range(2))
            ratio = sum((robot[axis]-first[axis])*segment[axis] for axis in range(2))/(length*length)
            ratio = max((begin-cumulative[index])/length, min((end-cumulative[index])/length, ratio))
            point = tuple(first[axis]+ratio*segment[axis] for axis in range(2))
            distance, arc = math.dist(robot, point), cumulative[index]+ratio*length
            if distance < best_distance-1e-9 or (abs(distance-best_distance) <= 1e-9 and arc < best_arc):
                best_distance, best_arc = distance, arc
        return best_arc

    def supported_turn(self, guide, cumulative, index):
        """用角点两侧实际轨迹的空间基线求方向，短 UWB 噪声段不能定义大转角。"""
        path, orders = guide.path, guide.orders
        predicted = math.dist(guide.predicted_target, guide.latest_target) > 1e-9
        observed_end = cumulative[-2] if predicted else cumulative[-1]
        before, after = cumulative[index], observed_end-cumulative[index]
        if before < self.corner_support-1e-9 or after < self.corner_support-1e-9:
            # 必须等待两侧各有足够真实空间长度；速度外推不能替尚未观察到的转角作证。
            # 原始拐点仍保留在 guide，新增有效历史后会重新判定，不能删除真实折返。
            return None
        incoming = self.fit_direction(path, orders, cumulative,
                                      cumulative[index]-self.corner_support, cumulative[index])
        outgoing = self.fit_direction(path, orders, cumulative,
                                      cumulative[index], cumulative[index]+self.corner_support)
        incoming_length, outgoing_length = math.hypot(*incoming), math.hypot(*outgoing)
        if min(incoming_length, outgoing_length) <= 1e-9:
            return None
        cosine = max(-1., min(1., sum(a*b for a, b in zip(incoming, outgoing))
                              /(incoming_length*outgoing_length)))
        return cosine, (('before_arc_m', self.corner_support), ('after_arc_m', self.corner_support),
                        ('before_fit_span_m', incoming_length), ('after_fit_span_m', outgoing_length),
                        ('angle_rad', math.acos(cosine)))

    def fit_direction(self, path, orders, cumulative, begin, end):
        """在有顺序的支持窗内拟合方向，分摊单个角点噪声，同时保留反向运动的符号。"""
        span = end-begin
        offsets = [span*index/6. for index in range(7)]
        points = [self.at_arc(path, orders, cumulative, begin+offset)[0] for offset in offsets]
        denominator = sum((offset-span*.5)**2 for offset in offsets)
        if denominator <= 1e-12:
            return (0., 0.)
        # 不能只连接“角点”和两侧端点：单点几厘米偏差会同时旋转两条方向。
        # 对弧长顺序做线性拟合，既抑制单点偏差，也避免无符号主方向把 180° 折返当共线。
        return tuple(span*sum((offset-span*.5)*point[axis] for offset, point in zip(offsets, points))
                     /denominator for axis in range(2))

    def build(self, guide, spacing, robot_position, preserve_corners=True):
        """沿有序进展形成有限前视目标；可保留拐点任务或允许地图平滑切弯。"""
        robot = FollowIntent._point(robot_position)
        if (guide is None or robot is None or not isinstance(spacing, (int, float))
                or not math.isfinite(spacing) or spacing < 0.):
            return None
        if guide.sequence != self._sequence:
            self.reset()
            self._sequence = guide.sequence
        path, orders = guide.path, guide.orders
        cumulative = self.cumulative(path)
        deleted_projection = self._order is not None and self._order < orders[0]-1e-9
        reanchored = not preserve_corners and (deleted_projection or self._history_lost)
        if reanchored:
            # 区域模式只需要人的最新尾段来形成目标，不执行狗尚未经过的历史检查点。
            # 删除旧投影后重新定位诊断，不能把无关的历史头部截断变成永久停车。
            self._order, self._robot, self._budget = None, None, self.projection_budget
        truncated = preserve_corners and (self._history_lost or deleted_projection)
        # 被删掉的未完成转角不能在下一次查询中凭空恢复；须由明确重置建立新意图。
        self._history_lost = truncated
        previous = 0. if self._order is None else self.arc_at_order(orders, cumulative, self._order)
        if reanchored:
            # 旧顺序锚点已不存在，应投影到仍真实观测的局部历史，而不是追逐远处的新头部。
            # 只在裁剪丢锚时重定位；正常自交/折返仍受累计进度预算限制，预测段不参与重定位。
            observed_end = cumulative[-1]
            if guide.prediction_seconds > 0. and len(path) > 1:
                observed_end = cumulative[-2]
            previous = self.nearest_arc(path,cumulative,robot,0.,observed_end)
        if self._robot is not None:
            # 投影预算来自真实机器人位移，连续空调用不能把进展推过整段 U 弯。
            # 转角处的圆滑执行使弧长可能大于直线位移，留 2 倍几何预算但严格限制累计余额。
            self._budget = min(self.projection_budget, self._budget+2.*math.dist(self._robot, robot))
        upper = min(cumulative[-1], previous+self._budget)
        projected = self.nearest_arc(path, cumulative, robot, previous, upper)
        projection, order = self.at_arc(path, orders, cumulative, projected)
        self._budget = max(0., self._budget-max(0., projected-previous))
        self._order, self._robot = order, robot
        desired_arc = max(0., cumulative[-1]-spacing)
        # 旧实验要求依序接近每个拐点；允许地图规划抄近时，这会与机器人实际路径产生死锁。
        # 两种模式均限制目标在当前进展前方：人已绕到后续路段时，不能直接抄到最新尾段。
        # 区域模式解除下面的逐拐点截断，允许在这段前视窗内平滑切弯，但保留路线分支顺序。
        goal_arc = max(projected,min(desired_arc,projected+self.lookahead))
        corner, corner_evidence = None, ()
        for index in range(1, len(path)-1):
            if cumulative[index] <= projected+1e-9 or cumulative[index] >= goal_arc:
                continue
            supported = self.supported_turn(guide, cumulative, index)
            if supported is None:
                continue
            cosine, evidence = supported
            if cosine < math.cos(self.corner_angle):
                if math.dist(robot, path[index]) <= FOLLOW_REGION_HALF_WIDTH+FOLLOW_GOAL_TOLERANCE:
                    # 搜索目标区域与 MPPI 终点容差共享完成口径，不能要求厘米级精确经过顶点。
                    continue
                # 前视可跨过拐点一点，使 MPPI 获得转向段；不能跨到尚未绕完的后续折返。
                corner = path[index]
                corner_evidence = evidence
                # 超过 120° 的折返后半段可能在几何上更近，先去折返点，避免终点抄掉整次折返。
                # 常规直角保留转角后的前视段，给优化器提前平滑转向的空间。
                extra = self.corner_lookahead if cosine > -.5 else 0.
                if preserve_corners:
                    goal_arc = min(goal_arc, cumulative[index]+extra)
                break
        goal, goal_order = self.at_arc(path, orders, cumulative, goal_arc)
        reference_path = [projection]
        reference_path.extend(point for point, arc in zip(path, cumulative)
                              if projected+1e-9 < arc < goal_arc-1e-9)
        if math.dist(reference_path[-1], goal) > 1e-9:
            reference_path.append(goal)
        virtual = guide.virtual_stem_available and projected < cumulative[1]-1e-9
        source = ('robot_virtual_stem' if virtual else 'short_velocity_prediction'
                  if goal_order > guide.target_stamp else 'observed_target_trail')
        return IntentReference(goal, tuple(reference_path), projection, math.dist(robot, projection), order,
                               max(0., desired_arc-projected), corner,
                               'HISTORY_TRUNCATED' if truncated else 'FRESH', source, guide.sequence,
                               guide.target_stamp, guide.age, guide.prediction_seconds, guide.observed_length, virtual,
                               corner_evidence)
