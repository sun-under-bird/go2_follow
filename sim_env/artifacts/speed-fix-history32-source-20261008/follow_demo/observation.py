"""有完成反馈的观察任务：接近、看向、等待新深度，不重复执行已观察姿态。"""
import copy
import math
import numpy as np
from .controller import wrap
from .navigation_config import OBSERVATION_POSITION_TOLERANCE, OBSERVATION_YAW_TOLERANCE
from .observation_geometry import (check_observation_position, observation_cell_state,
                                  observation_attempted)


class ObservationSession:
    """保持一次观察的姿态目标，实际朝向和新深度共同决定任务是否完成。"""
    def __init__(self):
        """初始化有界的已尝试视点和当前动作诊断，不产生任何底盘速度。"""
        self.plan = None
        self.stage = 'IDLE'
        self.tried = []
        self.result = ''
        self.gained_cells = 0
        self.region_check = {}
        self.relevant_gain = self.relevant_fresh_gain = 0
        self.relevant_free_gain = self.relevant_allowed_gain = 0
        self.evidence_since = None
        self.admitted_position = None
        self.sequence = 0
        self.last_outcome = None

    @staticmethod
    def known_cells(grid):
        """转成世界栅格编号比较观测增益，窗口平移不应被误认为新信息。"""
        rows,cols = np.nonzero(np.isfinite(grid.seen) | grid.occupied)
        offset = np.rint(grid.origin/grid.resolution).astype(int)
        return set(zip((cols+offset[0]).tolist(),(rows+offset[1]).tolist()))

    def begin(self, plan, pose, now, grid):
        """复制观察目标，隔离搜索器保存的旧路线；不会受下一轮评分小幅变化影响。"""
        self.plan = copy.deepcopy(plan)
        self.sequence += 1
        self.stage,self.result = 'MOVE',''
        self.started,self.progress_at = now,now
        self.frames_started = grid.frames
        self.best_error = abs(wrap(plan.look_yaw-pose.yaw))
        self.known_before = self.known_cells(grid)
        self.gained_cells = 0
        self.move_path = copy.deepcopy(plan.path)
        self.expected_cells = {tuple(cell) for cell in plan.observation_cells}
        self.relevant_before = observation_cell_state(grid,self.expected_cells,now)
        self.minimum_relevant_gain = max(3,math.ceil(.05*len(self.expected_cells-self.relevant_before['known'])))
        if grid.camera_observation:
            # 相机模式的任务是下一段包络 ROI；完成覆盖与候选准入的 30% 原前沿覆盖口径一致。
            self.minimum_relevant_gain = max(3,math.ceil(.30*len(self.expected_cells-self.relevant_before['known'])))
        self.region_check = {}
        self.relevant_gain = self.relevant_fresh_gain = 0
        self.relevant_free_gain = self.relevant_allowed_gain = 0
        self.evidence_since = None
        self.admitted_position = None

    def clear(self):
        """有可跟随通道或输入失效时释放当前任务，保留已看过的静态视点。"""
        self.plan,self.stage = None,'IDLE'

    def excluded(self, point, heading, resolution=.1):
        """排除同处、同朝向已执行过的视点，允许移动到不同位置重新观察。"""
        return observation_attempted(self.tried,point,heading,resolution)

    def finish(self, result, pose, grid):
        """记录真实结果及世界格增益，完成或失败都不能无限重发同一个观察姿态。"""
        self.result = result
        self.gained_cells = len(self.known_cells(grid)-self.known_before)
        # 下一次 begin 会清空当前结果；保留上次终止证据，低频验收采样也能看到真实原因。
        self.last_outcome = dict(sequence=self.sequence,result=result,t=pose.t,
                                 position=[pose.x,pose.y],yaw=pose.yaw,look_yaw=self.plan.look_yaw,
                                 stage=self.stage,evidence_since=self.evidence_since,
                                 relevant_fresh_gain=self.relevant_fresh_gain,
                                 relevant_free_gain=self.relevant_free_gain,
                                 relevant_allowed_gain=self.relevant_allowed_gain,
                                 minimum_relevant_gain=self.minimum_relevant_gain)
        self.tried.append([pose.x,pose.y,self.plan.look_yaw,copy.deepcopy(self.plan.observation_region)])
        self.tried = self.tried[-24:]
        self.clear()
        return result

    def inside_region(self, pose, grid):
        """以世界格判断实际位置进入已验证区域；不把单点容差扩大成观察成功。"""
        cell = grid.cell(pose.x,pose.y)
        return cell is not None and any(grid.cell(*point) == cell for point in self.plan.observation_region)

    def update_relevant_gain(self, grid, now, fresh_since):
        """按指定采集时间边界统计相关格实际新证据；移动和到位评估分别使用各自起点。"""
        current = observation_cell_state(grid,self.expected_cells,now)
        gained = current['known']-self.relevant_before['known']
        fresh = set()
        for col,row in gained:
            point = [(col+.5)*grid.resolution,(row+.5)*grid.resolution]
            cell = grid.cell(*point)
            # 已知集合可以包含持久障碍；仍须有本阶段真实采集时间，不能靠占用标志或未来时间过关。
            if cell is not None and fresh_since <= grid.seen[cell] <= now:
                fresh.add((col,row))
        self.evidence_since = fresh_since
        self.relevant_gain,self.relevant_fresh_gain = len(gained),len(fresh)
        self.relevant_free_gain = len(current['free']-self.relevant_before['free'])
        self.relevant_allowed_gain = len(current['allowed']-self.relevant_before['allowed'])

    def advance(self, pose, now, grid, parking_safe=True):
        """用实际位置验证观察区域、实际旋转和相关新深度推进；动作有明确失败出口。"""
        if self.plan is None:
            return None
        if now-self.started > 15.:
            return self.finish('OBSERVATION_TIMEOUT',pose,grid)
        if self.stage == 'MOVE':
            # 接近视点时相机一直在工作：原相关前沿若已获得足够新测量，信息任务已经完成。
            # 必须先核对实际证据，再判断剩余未知收益；否则 LOW_GAIN 恰好把成功揭示误当成无效位置。
            # 此分支只取消冗余看向，不宣告路线到达或通道可行；安全与下一段路线仍由导航器核验。
            self.update_relevant_gain(grid,now,self.started)
            fresh_frame = (grid.frames > self.frames_started and grid.last_depth is not None
                           and self.started <= grid.last_depth <= now)
            # 在真实相机任务中，几块新自由格可能仍无法容纳整机，不能因此撤销仍有效的移动路线。
            # 后台搜索一旦发现真实 FOLLOW 通道会自动替换本任务；信息计数本身只作为诊断。
            if (not grid.camera_observation and fresh_frame
                    and self.relevant_fresh_gain >= self.minimum_relevant_gain):
                return self.finish('OBSERVATION_COMPLETE',pose,grid)
        inside = self.inside_region(pose,grid)
        if inside:
            self.region_check = check_observation_position(grid,[pose.x,pose.y],self.plan.look_yaw,self.expected_cells,now,
                                                          initial_yaw=pose.yaw)
        else:
            self.region_check = dict(valid=False,reason='OUTSIDE_OBSERVATION_REGION')
        if self.stage == 'MOVE' and inside and self.region_check['valid'] and parking_safe:
            # 参考点只供 MPPI 接近；实际位置有安全余量及相同前沿收益，就不再追最后几厘米。
            self.plan.path = [[pose.x,pose.y]]
            self.admitted_position = [pose.x,pose.y]
            self.stage,self.progress_at = 'ORIENT',now
            self.best_error = abs(wrap(self.plan.look_yaw-pose.yaw))
            return 'OBSERVATION_ORIENT'
        if self.stage == 'MOVE':
            if inside and self.region_check['valid'] and not parking_safe:
                self.region_check['reason'] = 'OBSERVATION_STOPPING_SPACE'
            elif math.dist([pose.x,pose.y],self.move_path[-1]) <= OBSERVATION_POSITION_TOLERANCE:
                # 即使接近原参考点也必须有有效信息收益，不能退回旧的距离判据假完成。
                return self.finish('OBSERVATION_REGION_EXHAUSTED',pose,grid)
        if self.stage in ('ORIENT','EVALUATE'):
            related = observation_cell_state(grid,self.expected_cells,now)
            new_blockers = related['occupied']-self.relevant_before['occupied']
            # 新看到的墙会合理遮住原先预测的墙后 ROI，不能把获得障碍信息当作实际离开区域。
            # 只有相关格真实变成障碍才解释遮挡；没有这种证据时仍要求原前沿覆盖。
            self.region_check['observed_occlusion_cells'] = len(new_blockers)
            coverage_valid = self.region_check.get('overlap_ratio',0.) >= .30 or bool(new_blockers)
            # 看向和评估也复用已验证的矩形扫掠，不能再次用中心净空阈值把长方形近似成圆。
            geometry_valid = (inside and self.region_check.get('footprint_clear',False)
                              and coverage_valid and parking_safe)
            if not geometry_valid:
                # 残余位移离开有效区域时恢复移动参考；旧固定点的候选必须撤销。
                self.plan.path = copy.deepcopy(self.move_path)
                self.stage = 'MOVE'
                return 'OBSERVATION_REGION_LEFT'
        if self.stage == 'ORIENT':
            error = abs(wrap(self.plan.look_yaw-pose.yaw))
            if error < self.best_error-.08:
                self.best_error,self.progress_at = error,now
            if error <= OBSERVATION_YAW_TOLERANCE:
                self.stage,self.aligned_at,self.frames_at = 'EVALUATE',now,grid.frames
                return 'OBSERVATION_EVALUATE'
            if now-self.progress_at > 4.:
                # 原地小角速度响应不足不能靠加最低速度掩盖；标记失败并搜索另一有效视点。
                return self.finish('OBSERVATION_TURN_STALLED',pose,grid)
        if self.stage == 'EVALUATE':
            if abs(wrap(self.plan.look_yaw-pose.yaw)) > OBSERVATION_YAW_TOLERANCE:
                # 停车仍可能有残余转动；离开目标朝向后采到的新帧不能完成原观察任务。
                self.stage,self.progress_at = 'ORIENT',now
                self.best_error = abs(wrap(self.plan.look_yaw-pose.yaw))
                return 'OBSERVATION_ORIENT'
            fresh = grid.last_depth is not None and grid.last_depth >= self.aligned_at
            if now-self.aligned_at >= .4 and grid.frames >= self.frames_at+2 and fresh:
                self.update_relevant_gain(grid,now,self.aligned_at)
                if self.relevant_fresh_gain >= self.minimum_relevant_gain:
                    return self.finish('OBSERVATION_COMPLETE',pose,grid)
                if now-self.aligned_at >= 2.:
                    # 相机有新帧但需要的前沿仍未揭示，是一次无收益尝试，需要换视点。
                    return self.finish('OBSERVATION_NO_RELEVANT_GAIN',pose,grid)
        return None

    def diagnostics(self):
        """解释观察目前在哪一步、上次结果及排除视点数量，供人工验收。"""
        check = {key:value for key,value in self.region_check.items() if key not in ('unknown_cells','visible_cells')}
        return dict(stage=self.stage,result=self.result,sequence=self.sequence,gained_cells=self.gained_cells,tried_count=len(self.tried),
                    last_outcome=self.last_outcome,
                    region=[] if self.plan is None else self.plan.observation_region,
                    region_check=check,admitted_position=self.admitted_position,
                    evidence_since=self.evidence_since,
                    expected_cells=len(getattr(self,'expected_cells',())),relevant_gain=self.relevant_gain,
                    relevant_fresh_gain=self.relevant_fresh_gain,minimum_relevant_gain=getattr(self,'minimum_relevant_gain',3),
                    relevant_free_gain=self.relevant_free_gain,relevant_allowed_gain=self.relevant_allowed_gain)
