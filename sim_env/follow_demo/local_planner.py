"""已知通道上的滚动进展搜索；确实缺少可走通道时才选择观察任务。"""
from dataclasses import dataclass, field
import heapq
import math
import numpy as np
from .controller import wrap
from .navigation_config import (FOLLOW_REGION_HALF_WIDTH, OBSERVATION_POSITION_TOLERANCE,
                                MPPI_HORIZON_SECONDS, PLAN_UPDATE_SECONDS, FOLLOW_REFERENCE_SHORTFALL_COST)
from .observation_geometry import (OBSERVATION_DRIFT_MARGIN, OBSERVATION_MIN_GAIN,
                                   OBSERVATION_MIN_OVERLAP, OBSERVATION_REGION_RADIUS, observation_view,
                                   observation_gains, observation_masks, local_cell_mask, observation_attempted,
                                   camera_ground_view, observation_task_roi)


OBSERVATION_CANDIDATE_BUDGET = 32


@dataclass
class Plan:
    """一段可执行路线及其终点用途，观察目标不等于跟随目标。"""
    kind: str = 'WAITING'
    reason: str = 'NO_SAFE_CORRIDOR'
    path: list = field(default_factory=list)
    look_yaw: float = 0.0
    target: list = field(default_factory=list)
    observation_region: list = field(default_factory=list)
    observation_cells: list = field(default_factory=list)
    intent_reference: dict = field(default_factory=dict)
    search_cpu_ms: float = 0.0
    # 工作任务只返回私有搜索状态；控制线程接入有效计划后才提交换边偏好。
    search_side: int = 0
    search_pid: int = 0


class LocalPlanner:
    """在位置与八个朝向上搜索可达姿态，终点评分共享同一棵姿态搜索树。"""
    def __init__(self):
        """记住上次路线，避免观测小幅变化就左右换边。"""
        self.previous = Plan()
        self.side = 0

    def reachable(self, grid, free, pose, weights):
        """前进边检查平移矩形，转向边检查角点扫掠；二维可通行并集不能代替姿态图。"""
        poses,moves,turns,steps = grid.orientation_layers(free)
        start = grid.cell(pose.x,pose.y)
        distances = np.full(poses.shape,np.inf)
        parents,queue = {},[]
        self._seed_paths = {}
        center = grid.point(start)
        here = [pose.x,pose.y]

        def seed(cell, prefix):
            """把完整已知且姿态可达的连续前缀接入搜索树，不把连接范围加入自由证据。"""
            end = grid.point(cell)
            length = sum(math.dist(a,b) for a,b in zip(prefix,prefix[1:]))
            for heading in range(8):
                if not poses[heading][cell]:
                    continue
                angle = heading*math.pi/4
                cost = length*weights[cell]+.28*abs(wrap(angle-pose.yaw))
                node = (heading,*cell)
                if cost>=distances[node] or not grid.route_clear(free,prefix,pose.yaw,angle):
                    continue
                distances[node],parents[node] = cost,None
                self._seed_paths[node] = prefix
                heapq.heappush(queue,(cost,node))

        seed(start,[here,center])
        # 实际姿态可能位于离散姿态之间：不能在没有原地转身空间时宣称整个通道不可达。
        # 先尝试直接连接前方格，再尝试沿当前朝向移动到有转向空间处；每条前缀均全包络验证。
        for distance in (.2,.4,.6,.8,1.):
            point = [pose.x+distance*math.cos(pose.yaw),pose.y+distance*math.sin(pose.yaw)]
            continuous_safe = grid.motion_clear(free,[*here,pose.yaw],[*point,pose.yaw])
            candidate = grid.cell(*point)
            if candidate is None:
                break
            cells = [(candidate[0]+dr,candidate[1]+dc) for dr in (-1,0,1) for dc in (-1,0,1)
                     if 0<=candidate[0]+dr<grid.size and 0<=candidate[1]+dc<grid.size]
            cells.sort(key=lambda cell:math.dist(grid.point(cell),point))
            for cell in cells[:4]:
                end = grid.point(cell)
                seed(cell,[here,end])
                if continuous_safe:
                    seed(cell,[here,point,end])
        while queue:
            cost,node = heapq.heappop(queue)
            if cost > distances[node]:
                continue
            heading,row,col = node
            dr,dc = steps[heading]
            transitions = []
            if moves[heading,row,col]:
                other = (heading,row+dr,col+dc)
                if 0 <= other[1] < grid.size and 0 <= other[2] < grid.size:
                    transitions.append((other,grid.resolution*math.hypot(dr,dc)*weights[other[1:]]))
            if turns[heading,row,col]:
                transitions.append((((heading+1)%8,row,col),.28*math.pi/4))
            if turns[(heading-1)%8,row,col]:
                transitions.append((((heading-1)%8,row,col),.28*math.pi/4))
            for other,edge in transitions:
                proposed = cost+edge
                if proposed < distances[other]:
                    distances[other],parents[other] = proposed,node
                    heapq.heappush(queue,(proposed,other))
        best_heading = np.argmin(distances,axis=0)
        best = np.min(distances,axis=0)
        self._search_states = {tuple(cell):(int(best_heading[tuple(cell)]),*map(int,cell))
                               for cell in np.argwhere(np.isfinite(best))}
        return best,parents

    def search(self, grid, allowed, clearance, pose, target, velocity, spacing, tried=(), intent_reference=None,
               force_observation=False):
        """依次搜索舒适距离带、已知通道进展和观察姿态，所有执行路径均来自安全连通区。"""
        start = grid.cell(pose.x, pose.y)
        free,_,_ = grid.layers(pose.t)
        if start is None or not grid.pose_clear(free,pose.x,pose.y,pose.yaw):
            return Plan(reason='FOOTPRINT_UNOBSERVED')
        distance_from_body = clearance.astype(float)-grid.radius
        margin = distance_from_body-grid.resolution*math.sqrt(.5)
        # 净空只取决于目的格，提前计算同一因子，避免八邻接内重复标量 max/sqrt。
        # 仍保留目的格有向代价及原 allowed，不能借提速放松未知与斜穿角的边界。
        weights = 1.+.18/np.maximum(.06,margin)+2.*np.maximum(0.,.15-margin)
        distances,parents = self.reachable(grid,free,pose,weights)
        cells = np.argwhere(np.isfinite(distances))
        if not len(cells):
            return Plan(reason='NO_REACHABLE_RECTANGLE_POSE')
        points = grid.origin + (cells[:, ::-1] + 0.5) * grid.resolution
        travel = distances[cells[:, 0], cells[:, 1]]
        target = np.asarray(target)
        speed = np.linalg.norm(velocity)
        direction = np.asarray(velocity) / speed if speed > 0.15 else target - [pose.x, pose.y]
        direction = direction / max(np.linalg.norm(direction), 1e-6)
        desired = target - spacing * direction
        separation = np.linalg.norm(points - target, axis=1)
        if intent_reference is None:
            goals = np.abs(separation - spacing) <= FOLLOW_REGION_HALF_WIDTH
            # 直线间距合适但与人隔着已知障碍，不是完成跟随的终点。
            # 否则人绕过墙后转弯时会把终点拉回墙的近侧，机器人先转错方向再掉头。
            # 未知并不等于墙；这里只排除已知占据格，执行路线仍须完整位于已知自由区域。
            indices = np.flatnonzero(goals)
            goals[indices] &= ~self.wall_occluded(grid,points[indices],target)
        else:
            # 试验分支仅改“去哪里”的语义：围绕有顺序的滚动参考选局部目标。
            # 人的轨迹没有加入 allowed 或搜索边，机器人路径仍由同一 Dijkstra 树重建。
            if intent_reference.status != 'FRESH':
                return Plan(reason='FOLLOW_TRAIL_HISTORY_LOST')
            target = desired = np.asarray(intent_reference.goal)
            separation = np.linalg.norm(points-target, axis=1)
            goals = separation <= FOLLOW_REGION_HALF_WIDTH
        target_bearing = math.atan2(target[1] - pose.y, target[0] - pose.x)
        end_margin = margin[cells[:,0],cells[:,1]]
        if force_observation:
            # 路径几何可走但执行预测会停进无出口的口袋时，主动观察而非等待进展超时。
            # 观察仍须通过原位置区域、矩形旋转、CameraInfo/TF及相关新格验证。
            observed=self.search_observation(grid,allowed,clearance,pose,target,cells,points,travel,
                                             separation,parents,start,target_bearing,tried,weights)
            if intent_reference is not None:
                observed.intent_reference=intent_reference.diagnostics()
            return observed
        if np.any(goals):
            # 首要任务是沿现有通道接近跟随区域；刚转弯时不能为抢占人的正后方反向绕圈。
            score = 1.3 * travel + 0.30 * np.linalg.norm(points - desired, axis=1)
            if intent_reference is None and speed > .15:
                # 人仍在走时，零长度参考会让 FollowPath 立即成功并反复清零平滑状态。
                # 在同一安全距离带内优先供给可持续跟踪的长度；不足时仍允许短路线，不能因此去未知区。
                # 只计沿人的运动方向的进展；向反方向拉长路线会在刚转弯时造成反向绕圈。
                supplied = np.maximum(0.,(points-[pose.x,pose.y])@direction)
                reference_length = min(speed*(MPPI_HORIZON_SECONDS+PLAN_UPDATE_SECONDS),
                                       math.dist([pose.x,pose.y],desired))
                score += FOLLOW_REFERENCE_SHORTFALL_COST*np.maximum(0.,reference_length-supplied)
                # 平行跟随时偏向人的运动线，抑制宽距离带两侧端点交替而产生的无谓转向。
                # 只是软代价：绕障路径仍由同一可达树及矩形扫掠决定。
                offset = points-desired
                score += .8*np.abs(offset[:,0]*direction[1]-offset[:,1]*direction[0])
            # 不为满足精确间距把终点放到墙角边缘；净空是软偏好，窄通道仍可通行。
            score += .22/np.maximum(.08,end_margin+.08)
            if self.previous.path:
                score += 0.25 * np.linalg.norm(points - self.previous.path[-1], axis=1)
            score[~goals] = np.inf
            selected = int(np.argmin(score))
            reason = 'FOLLOW_TRAIL_REFERENCE' if intent_reference is not None else 'FOLLOW_REGION_REACHABLE'
            result = Plan('FOLLOWING', reason, target=target.tolist())
        else:
            # 人的舒适距离带暂时不可达，不代表已看清的前方通道不能继续走。
            # 先选安全连通区内的滚动进展终点；未知只决定后续观察需求，绝不加入执行路径。
            selected = self.progress_candidate(points,travel,separation,pose,target,end_margin,speed)
            if selected is not None:
                reason = 'KNOWN_TRAIL_PROGRESS' if intent_reference is not None else 'KNOWN_CORRIDOR_PROGRESS'
                result = Plan('FOLLOWING',reason,target=target.tolist())
            else:
                result = self.search_observation(grid,allowed,clearance,pose,target,cells,points,travel,
                                                 separation,parents,start,target_bearing,tried,weights)
                if intent_reference is not None:
                    result.intent_reference = intent_reference.diagnostics()
                return result
        result = self.make_path(grid,allowed,pose,result,cells,parents,start,selected,target_bearing,weights)
        if intent_reference is not None:
            result.intent_reference = intent_reference.diagnostics()
        return result

    def search_observation(self, grid, allowed, clearance, pose, target, cells, points, travel,
                           separation, parents, start, target_bearing, tried, weights):
        """通道无法取得足够进展时才寻找观察姿态，保持真实自由空间的执行边界。"""
        camera_observation = getattr(grid, 'camera_observation', False)
        camera_model = getattr(grid, 'camera_model', None)
        if camera_observation and camera_model is None:
            return Plan(reason='CAMERA_GEOMETRY_UNAVAILABLE')
        # 观察候选也是已知自由区域内的机器人中心，不能把未知格子作为可行驶终点。
        candidates = self.observation_candidates(grid,cells,points,travel,separation,clearance,pose)
        if not len(candidates):
            # 路径仍可使用原 allowed；只有停下转向的位置需要额外漂移空间。
            return Plan(reason='NO_ROBUST_OBSERVATION')
        best, selected, look = math.inf, None, pose.yaw
        free, _, _ = grid.layers(pose.t)
        views,view_points,view_headings = [],[],[]
        for index in candidates:
            point = points[index]
            bearing = math.atan2(target[1] - point[1], target[0] - point[0])
            travel_heading = math.atan2(point[1]-pose.y,point[0]-pose.x)
            # 当前执行器不主动倒退：观察位置在身后时必须支付转身、再转回观察的代价。
            # 忽略这一点会在几乎到达后反复选择后方几十厘米处，把连续通道变成掉头任务。
            move_turn = (abs(wrap(travel_heading-pose.yaw))
                         if math.dist(point,[pose.x,pose.y]) > OBSERVATION_POSITION_TOLERANCE else 0.)
            for heading in (bearing, bearing + 0.8, bearing - 0.8, pose.yaw):
                arrival = self._search_states[tuple(cells[index])][0]*math.pi/4
                if not grid.motion_clear(free,[*point,arrival],[*point,heading],OBSERVATION_DRIFT_MARGIN):
                    continue
                if observation_attempted(tried,point,heading,grid.resolution):
                    continue
                views.append((index,bearing,heading,move_turn))
                view_points.append(point)
                view_headings.append(heading)
        # 候选评分只要数值收益，批量计算中不创建数百个世界格 tuple/set。
        if camera_observation:
            # 每个视点只查询通向当前跟随终点的下一段包络；四种朝向共用同一任务 ROI。
            # 不按全视野未知面积奖励，侧向/远离目标的视点仍可凭真实相关可见性胜出。
            rois = {int(index): observation_task_roi(grid, free, points[index], target)
                    for index in candidates}
            gains = np.array([camera_ground_view(grid, free, point, heading, camera_model,
                                                 include_visible=False, selected_cells=rois[index])['roi_unknown_coverage']
                              for (index, _, heading, _), point in zip(views, view_points)])
            minimum_gain = max(OBSERVATION_MIN_GAIN, OBSERVATION_MIN_OVERLAP)
        else:
            gains = observation_gains(grid,free,view_points,view_headings)
            minimum_gain = OBSERVATION_MIN_GAIN
        for (index,bearing,heading,move_turn),gain in zip(views,gains):
            if gain < minimum_gain:
                continue
            point = points[index]
            # 只追求未知面积会转去观察与通行无关的一侧；向目标方向的视野优先。
            # 侧向仍保留收益，允许绕墙端点，目标方位不直接抢占执行转向。
            useful_gain = gain*(.35+.65*max(0.,math.cos(wrap(heading-bearing))))
            lateral = math.sin(wrap(math.atan2(point[1]-pose.y,point[0]-pose.x)-target_bearing))
            switch_cost = 0.7 if self.side and lateral*self.side < -0.2 else 0.0
            score = 0.35*travel[index]+0.65*separation[index]-2.2*useful_gain+0.15*abs(wrap(heading-pose.yaw))+.45*move_turn+switch_cost
            if self.previous.kind == 'OBSERVING' and self.previous.path:
                score += 0.35*math.dist(point,self.previous.path[-1])
            if score < best:
                best,selected,look = score,int(index),heading
        if selected is None:
            return Plan(reason='NO_USEFUL_OBSERVATION')
        result = Plan('OBSERVING', 'GOAL_UNOBSERVED', look_yaw=wrap(look), target=target.tolist())
        if camera_observation:
            # 任务保留整段缺失包络，不能将未投影格裁掉，再虚构该任务已完全覆盖。
            expected = rois[selected]
        else:
            expected = observation_view(grid,free,points[selected],result.look_yaw)['unknown_cells']
        result.observation_cells = [list(cell) for cell in sorted(expected)]
        result.observation_region = self.observation_region(grid,allowed,clearance,cells,points,
                                                            points[selected],result.look_yaw,expected,free)
        if not result.observation_region:
            return Plan(reason='NO_ROBUST_OBSERVATION')
        return self.make_path(grid,allowed,pose,result,cells,parents,start,selected,target_bearing,weights)

    def make_path(self, grid, allowed, pose, result, cells, parents, start, selected, target_bearing, weights):
        """沿已知安全搜索树回溯并简化路径，跟随和观察共享同一包络校验。"""
        node = self._search_states[tuple(cells[selected])]
        route = [grid.point(node[1:])]
        while parents[node] is not None:
            node = parents[node]
            point = grid.point(node[1:])
            if math.dist(point,route[-1]) > 1e-9:
                route.append(point)
        route.reverse()
        route = self._seed_paths[node][:-1]+route
        free,_,_ = grid.layers(pose.t)
        final_yaw = result.look_yaw if result.kind == 'OBSERVING' else None
        here = [pose.x,pose.y]
        connected = None
        for index in range(min(len(route),9)):
            outgoing = (math.atan2(route[index+1][1]-route[index][1],route[index+1][0]-route[index][0])
                        if index+1<len(route) else final_yaw)
            # 连续起点不能强接最近格中心；在窄通道里这会制造并不必要的45°转身。
            # 只接入经过完整平移与两端转向验证的前方点，不跨墙抄近路。
            if grid.route_clear(free,[here,route[index]],pose.yaw,outgoing):
                connected = [here]+route[index:]
                break
        if connected is None:
            return Plan(reason='PATH_CONNECTION_RECTANGLE_BLOCKED')
        route = connected
        compact = [route[0]]
        for point in route[1:]:
            if math.dist(compact[-1],point)>1e-9:
                compact.append(point)
        route = compact
        if not grid.route_clear(free,route,pose.yaw,final_yaw):
            return Plan(reason='PATH_RECTANGLE_SWEEP_BLOCKED')
        # 用同一口径的线段离散代价比较原路线和候选捷径。
        # 只查不碰障碍会把 Dijkstra 的宽通道重新拉直到墙角，抹掉搜索的净空偏好。
        cumulative = np.zeros(len(route))
        edge_peak = np.zeros(max(0,len(route)-1))
        for j,(a,b) in enumerate(zip(route,route[1:])):
            cost,edge_peak[j] = self.segment_metrics(grid,free,weights,a,b)
            cumulative[j+1] = cumulative[j]+cost
        if not np.isfinite(cumulative[-1]):
            return Plan(reason='PATH_CONNECTION_UNKNOWN')
        result.path = [route[0]]
        index = 0
        while index < len(route) - 1:
            end = len(route) - 1
            while end > index+1:
                original = cumulative[end]-cumulative[index]
                shortcut,peak = self.segment_metrics(grid,free,weights,route[index],route[end])
                entering = (pose.yaw if len(result.path) == 1 else
                            math.atan2(result.path[-1][1]-result.path[-2][1],result.path[-1][0]-result.path[-2][0]))
                exiting = (math.atan2(route[end+1][1]-route[end][1],route[end+1][0]-route[end][0])
                           if end+1 < len(route) else final_yaw)
                # 拉直以后，两端新的朝向变化也要能转过去；只检查直线矩形会漏掉转角扫墙。
                if not grid.route_clear(free,[route[index],route[end]],entering,exiting):
                    shortcut = math.inf
                # 在原路线具备空间时保留至少 0.20 m 舒适余量；本来就窄的通道不被切断。
                # 只限制后处理不能把通道挤窄，不把这项舒适偏好变成新的全图硬膨胀。
                peak_limit = max(1.+.18/.20,float(np.max(edge_peak[index:end])))
                if shortcut <= original*1.005+1e-9 and peak <= peak_limit+1e-9:
                    break
                end -= 1
            result.path.append(route[end])
            index = end
        if result.path and len(result.path) > 1:
            first = result.path[1]
            lateral = math.sin(wrap(math.atan2(first[1] - pose.y, first[0] - pose.x) - target_bearing))
            if abs(lateral) > 0.3:
                self.side = 1 if lateral > 0 else -1
        self.previous = result
        return result

    @staticmethod
    def wall_occluded(grid, points, target):
        """批量判断终点到人的连线是否接触已知障碍格；窗口外目标也可判断窗口内的墙。"""
        points = np.asarray(points,dtype=float).reshape(-1,2)
        occupied = np.argwhere(grid.occupied)
        result = np.zeros(len(points),dtype=bool)
        if not len(points) or not len(occupied):
            return result
        low = grid.origin+occupied[:,::-1]*grid.resolution
        high = low+grid.resolution
        # 用闭栅格方盒与线段的参数区间求交，包含擦边和擦角；不能用稀疏射线采样漏掉薄墙。
        # 分块只控制临时数组大小，不限制墙的数目或连线长度。
        for begin in range(0,len(points),64):
            starts = points[begin:begin+64,None,:]
            delta = np.asarray(target)-starts
            parallel = np.abs(delta)<1e-12
            divisor = np.where(parallel,1.,delta)
            first,last = (low-starts)/divisor,(high-starts)/divisor
            near,far = np.minimum(first,last),np.maximum(first,last)
            inside = (starts>=low-1e-12)&(starts<=high+1e-12)
            near = np.where(parallel,np.where(inside,-np.inf,np.inf),near)
            far = np.where(parallel,np.where(inside,np.inf,-np.inf),far)
            enter = np.maximum(0.,np.max(near,axis=2))
            leave = np.minimum(1.,np.min(far,axis=2))
            result[begin:begin+64] = np.any(enter<=leave+1e-12,axis=1)
        return result

    @staticmethod
    def segment_metrics(grid, free, weights, a, b):
        """返回线段代理代价与最差格权重，总代价和局部净空均不能被拉直抹掉。"""
        cells = grid.segment_cells(a,b)
        if cells is None:
            return math.inf,math.inf
        indices = np.asarray(cells,dtype=int)
        heading = math.atan2(b[1]-a[1],b[0]-a[0])
        if not grid.motion_clear(free,[*a,heading],[*b,heading]):
            return math.inf,math.inf
        # 这是统一的离散代理代价，触角格按平均权重计入，不能当作精确连续积分。
        values = weights[indices[:,0],indices[:,1]]
        return math.dist(a,b)*float(np.mean(values)),float(np.max(values))

    def progress_candidate(self, points, travel, separation, pose, target, margin, target_speed=0.):
        """选择有实际前进长度的已知通道终点，避免落后后进入更慢的观察循环。"""
        offsets = points-[pose.x,pose.y]
        length = np.linalg.norm(offsets,axis=1)
        current_separation = math.dist([pose.x,pose.y],target)
        relative = np.arctan2(offsets[:,1],offsets[:,0])-pose.yaw
        angles = np.abs((relative+math.pi)%(2*math.pi)-math.pi)
        # 至少留出一段移动参考，过滤厘米级边界点；长度不足时停车观察仍是正常出口。
        # 前向锥仅约束这个过渡目标，目标在身后且缺少已知路线时先观察，禁止盲退。
        candidates = (length >= .8) & (travel <= 3.8) & (angles <= 1.05)
        candidates &= separation <= current_separation-.35
        if not np.any(candidates):
            return None
        score = separation+.20*travel+.15*angles+.22/np.maximum(.08,margin+.08)
        # 距离区域不可达时同样供给一个预测窗口，避免落后后不断选择更短、更慢的路径。
        # 加权预算仍限制复杂绕行；参考长度是软偏好，不把未知区域加入可通行集合。
        reference = min(target_speed*(MPPI_HORIZON_SECONDS+PLAN_UPDATE_SECONDS),
                        max(0.,current_separation-1.8))
        direction = (np.asarray(target)-[pose.x,pose.y])/max(current_separation,1e-6)
        supplied = np.maximum(0.,offsets@direction)
        lateral = np.abs(offsets[:,0]*direction[1]-offsets[:,1]*direction[0])
        score += FOLLOW_REFERENCE_SHORTFALL_COST*np.maximum(0.,reference-supplied)+.8*lateral
        if self.previous.kind == 'FOLLOWING' and self.previous.path:
            score += .12*np.linalg.norm(points-self.previous.path[-1],axis=1)
        score[~candidates] = np.inf
        return int(np.argmin(score))

    @staticmethod
    def observation_region(grid, allowed, clearance, cells, points, center, heading, expected_cells, free):
        """构造同一视向下有足够收益且覆盖原前沿的连通位置集合，不用距离容差代替观察。"""
        expected_cells = set(map(tuple, expected_cells))
        nearby = np.flatnonzero(np.linalg.norm(points-center,axis=1) <= OBSERVATION_REGION_RADIUS)
        nearby = np.array([index for index in nearby
                           if grid.pose_clear(free,*points[index],heading,OBSERVATION_DRIFT_MARGIN)],dtype=int)
        region = set()
        expected_mask = local_cell_mask(grid,expected_cells)
        # 区域仍逐格检验相同物理视野，内部直接与mask相交；分母包含窗口外相关格。
        if getattr(grid, 'camera_observation', False):
            # 搜索中心和实际区域准入共用 CameraInfo/TF 地面支持，避免中心正确、宽区域假准入。
            for index in nearby:
                view = camera_ground_view(grid, free, points[index], heading, getattr(grid, 'camera_model', None),
                                          selected_cells=expected_cells)
                overlap = len(view['visible_cells'] & expected_cells) / max(1, len(expected_cells))
                if (view['available'] and view['roi_unknown_coverage'] >= OBSERVATION_MIN_GAIN
                        and overlap >= OBSERVATION_MIN_OVERLAP):
                    region.add(tuple(cells[index]))
        else:
            for begin in range(0,len(nearby),16):
                indices = nearby[begin:begin+16]
                views = observation_masks(grid,free,points[indices],np.full(len(indices),heading))
                overlap = np.count_nonzero(views['visible_masks'] & expected_mask,axis=1)/max(1,len(expected_cells))
                valid = (views['gains'] >= OBSERVATION_MIN_GAIN) & (overlap >= OBSERVATION_MIN_OVERLAP)
                region.update(map(tuple,cells[indices[valid]]))
        start = grid.cell(*center)
        if start not in region:
            return []
        # 邻近距离不能证明连通；仍以原安全格和禁止斜穿角的规则保留中心所在分量。
        reached,queue = {start},[start]
        while queue:
            row,col = queue.pop()
            for dr in (-1,0,1):
                for dc in (-1,0,1):
                    other = row+dr,col+dc
                    if not (dr or dc) or other not in region or other in reached:
                        continue
                    if dr and dc and not (allowed[row+dr,col] and allowed[row,col+dc]):
                        continue
                    reached.add(other)
                    queue.append(other)
        return [grid.point(cell) for cell in sorted(reached)]

    @staticmethod
    def observation_candidates(grid, cells, points, travel, separation, clearance, pose):
        """细化粗采样附近与近身视点，限定收益评估预算，同时排除不稳定的转向边缘。"""
        local_clearance = clearance[cells[:,0],cells[:,1]]
        free,_,_ = grid.layers(pose.t)
        robust_map = np.any(grid.orientation_layers(free,OBSERVATION_DRIFT_MARGIN)[0],axis=0)
        robust = np.flatnonzero((travel < 4.) & robust_map[cells[:,0],cells[:,1]])
        if not len(robust):
            return np.array([],dtype=int)
        distance = np.linalg.norm(points-[pose.x,pose.y],axis=1)
        priority = .35*travel+.65*separation
        coarse = robust[(cells[robust,0] % 3 == 0) & (cells[robust,1] % 3 == 0)]
        # 十六个粗网格代表先占位，保留绕墙两侧与较远通道；细化点不能再次挤掉它们。
        seeds = coarse[np.argsort(priority[coarse],kind='stable')[:16]]
        nearest = int(robust[np.argmin(distance[robust])])
        nearby = robust[distance[robust] <= .5]
        # 当前位置只在满足漂移余量时保留；附近高净空格不能因 mod3 相位被遗漏。
        # 最近点、四个近身前沿与三个高净空点最多占八席；限制局部密集采样的份额。
        near_priority = nearby[np.argsort(priority[nearby],kind='stable')[:4]]
        near_stable = nearby[np.lexsort((distance[nearby],-local_clearance[nearby]))[:3]]
        local = np.unique(np.concatenate(([nearest],near_priority,near_stable)))
        required = np.unique(np.concatenate((local,seeds)))
        expanded = set(required.tolist()) | set(nearby.tolist())
        lookup = {tuple(cell):index for index,cell in enumerate(cells)}
        robust_set = set(robust.tolist())
        # 0.30 m 粗采样的相邻格加入 0.10 m 精细视点，始终只取原安全连通区。
        for index in seeds:
            row,col = cells[index]
            for dr in (-1,0,1):
                for dc in (-1,0,1):
                    candidate = lookup.get((row+dr,col+dc))
                    if candidate in robust_set:
                        expanded.add(candidate)
        remaining = np.array(sorted(expanded-set(required.tolist())),dtype=int)
        order = remaining[np.argsort(priority[remaining],kind='stable')]
        return np.concatenate((required,order[:OBSERVATION_CANDIDATE_BUDGET-len(required)]))

    @staticmethod
    def information_gain(grid, free, point, heading):
        """估计前视扇区可能新增的未知区域；射线到已知障碍终止，不读取真实场景。"""
        return float(observation_gains(grid,free,[point],[heading])[0])
