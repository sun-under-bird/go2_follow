"""静态平地连续跟随：异步通道搜索、Nav2 MPPI 输入和统一执行保护。"""
import copy
import math
import multiprocessing
import os
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import numpy as np
from .controller import FollowController, Pose, clamp, wrap
from .heading_guide import heading_diagnostics
from .local_map import RollingMap
from .local_planner import LocalPlanner, Plan
from .navigation_config import (FOLLOW_REGION_HALF_WIDTH, FOLLOW_GOAL_TOLERANCE, MAX_NAVIGATION_TURN,
                                MAX_NAVIGATION_SPEED, MPPI_HORIZON_SECONDS, PLAN_UPDATE_SECONDS,
                                FACE_ENTER_ANGLE,FACE_EXIT_ANGLE,FACE_STATIONARY_SECONDS,
                                BRAKE_DECELERATION, CONTROL_HOLD_SECONDS, execution_hold_seconds)
from .navigation_config import FOLLOW_HISTORY_LENGTH, FOLLOW_HISTORY_POINTS
from .observation import ObservationSession
from .follow_intent import FollowIntent, TrailReferenceTracker
from .speed_reference import route_speed_reference
from .native_braking import NativeBraking
from .command_smoothing import smooth_axis


def initialize_search_worker():
    """限制新进程的 OpenCV 并行度；spawn 不继承父进程已经设置的线程配置。"""
    import cv2
    cv2.setNumThreads(1)


def search_worker_pid():
    """返回工作进程身份，初始化时预热 spawn，避免第一份地图任务承担导入耗时。"""
    return os.getpid()


def search_plan(snapshot, pose, target, velocity, spacing, requested, tried,
                intent_reference=None, use_follow_intent=False, previous=None, side=0, force_observation=False):
    """用独立快照搜索并返回原三元组；不持有导航器、ROS 节点、池或锁。"""
    started, cpu_started = time.monotonic(), time.thread_time()
    # 每次调用恢复已提交状态，不把工作进程中被拒收的旧结果作为下一次规划偏好。
    planner = LocalPlanner()
    planner.previous = Plan() if previous is None else previous
    planner.side = side
    if use_follow_intent and intent_reference is None:
        plan = Plan(reason='FOLLOW_TRAIL_INVALID')
    else:
        _, allowed, clearance = snapshot.layers(pose.t)
        # 包络与未知区域仍为硬门槛；进程隔离仅改变调度，不改变地图证据或安全阈值。
        plan = planner.search(snapshot, allowed, clearance, pose, target, velocity, spacing, tried,
                              intent_reference=intent_reference,force_observation=force_observation)
    plan.search_cpu_ms = (time.thread_time()-cpu_started)*1000
    plan.search_side, plan.search_pid = planner.side, os.getpid()
    return plan, (time.monotonic()-started)*1000, requested


class NavigationController(FollowController):
    """控制线程持有地图，搜索读取副本，MPPI 只提供待检查的候选命令。"""
    def __init__(self, use_follow_intent=False, use_camera_observation=False, use_process_search=False):
        """明确使用静态历史地图；不将盲区历史证据当作动态侵入安全保证。"""
        super().__init__()
        self.max_speed,self.max_turn = MAX_NAVIGATION_SPEED,MAX_NAVIGATION_TURN
        self.grid = RollingMap(static_history=True)
        self.native_braking = NativeBraking()
        self.grid.camera_observation = bool(use_camera_observation)
        self.planner, self.plan = LocalPlanner(), Plan()
        self.use_process_search = bool(use_process_search)
        # 显式 spawn 不继承主进程的 DDS、OpenGL 及多线程锁，地图只以私有副本进入工作进程。
        self.pool = (ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                         initializer=initialize_search_worker)
                     if self.use_process_search else
                     ThreadPoolExecutor(max_workers=1, thread_name_prefix='follow-search'))
        self.search_closed, self.search_failed = False, False
        self.search_error = ''
        self.future = None
        self.search_worker_pids = set()
        if self.use_process_search:
            # 此时尚未启动控制/物理步进；只预热进程，不提交地图任务，也不放宽结果年龄。
            try:
                self.search_worker_pids.add(self.pool.submit(search_worker_pid).result(timeout=10.))
            except Exception:
                self.search_closed = True
                self.pool.shutdown(wait=True, cancel_futures=True)
                raise
        self.last_plan, self.plan_ms, self.plan_revision = -math.inf, 0., 0
        self.plan_cpu_ms = 0.
        self.minimum_plan_stamp = -math.inf
        self.motion, self.phase = (0., 0., 0.), 'FOLLOWING'
        self.control_pose = None
        self.stall_since, self.progress_pose = None, None
        self.progress_failed = False
        self.observing_since = None
        self.safety_path, self.footprint = [], {}
        self.depth_timeout, self.code = .9, 'WAITING'
        self.confirmation_requested = False
        self.external_command, self.external_stamp = (0., 0.), -math.inf
        self.external_active = False
        self.speed_limit, self.safety_limited = 0., False
        self.speed_reference = dict(speed=0., reason='WAITING')
        self.tracking_requested = False
        self.observation = ObservationSession()
        self.rejected_plan_count,self.rejected_plan_reason = 0,''
        self.guard_diagnostic = {}
        self.trajectory_diagnostic = {}
        self.smoothing_diagnostic = {}
        self.continuation_limited = False
        self.halt_counts, self.halt_events = {}, []
        self.stationary_since,self.face_active = None,False
        self.face_diagnostic = dict(status='IDLE')
        self.heading_diagnostic = {}
        self.use_follow_intent = bool(use_follow_intent)
        # 有序意图的预测长度也覆盖控制窗口，避免在完成转角顺序后重新变成短参考停车。
        # 预测只形成有限意图；其未知区域和拐角仍须经过地图搜索与执行保护。
        self.follow_intent = (FollowIntent(max_prediction=MPPI_HORIZON_SECONDS+PLAN_UPDATE_SECONDS+.2,
                                          max_history_length=FOLLOW_HISTORY_LENGTH,max_points=FOLLOW_HISTORY_POINTS)
                              if self.use_follow_intent else None)
        self.intent_tracker = TrailReferenceTracker() if self.use_follow_intent else None
        self.intent_reference = None
        self.intent_diagnostic = dict(enabled=self.use_follow_intent, status='WAITING')

    def observe(self, local_x, local_y, stamp):
        """旧 UWB 门控成功后才记录世界轨迹，不重复加入控制周期中的目标外推点。"""
        accepted = super().observe(local_x, local_y, stamp)
        if accepted and self.use_follow_intent:
            pose = self.history.at(stamp)
            self.follow_intent.update(self.last_world_measurement, stamp,
                                      robot_position=(pose.x, pose.y), velocity=self.target_velocity)
        return accepted

    def prepare_intent_reference(self, now, pose, spacing):
        """在控制线程形成不可变参考，异步搜索只能读取本次快照，不能改变轨迹进展。"""
        if not self.use_follow_intent:
            return None
        guide = self.follow_intent.guide(now, prediction_horizon=MPPI_HORIZON_SECONDS+PLAN_UPDATE_SECONDS)
        reference = self.intent_tracker.build(guide,spacing,(pose.x,pose.y),preserve_corners=False)
        self.intent_reference = reference
        self.intent_diagnostic = dict(enabled=True, history=self.follow_intent.diagnostics())
        if reference is None:
            self.intent_diagnostic['status'] = 'TARGET_INVALID'
        else:
            self.intent_diagnostic.update(reference.diagnostics(), latest_target=list(guide.latest_target),
                                          predicted_target=list(guide.predicted_target),
                                          corner_policy='progress_window_without_mandatory_waypoint')
        return reference

    def close(self):
        """幂等回收唯一搜索线程或进程；运行中的任务须退出，重置不能遗留后台搜索。"""
        if self.search_closed:
            return
        self.search_closed = True
        self.native_braking.close()
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.future = None

    def confirm_start(self, now):
        """只在里程计新鲜时接受本轮一次起始净空确认。"""
        pose = self.history.current(now)
        if pose is None:
            return False
        return self.grid.confirm_start(pose.x, pose.y, now)

    def halt(self, state, code, reason, now=None):
        """统一清空输出和舒适性积分，硬保护优先于速度平滑。"""
        if self.state != state or self.code != code:
            stamp = self.last_tick if now is None else now
            pose_stamp = self.history.values[-1].t if self.history.values else None
            self.halt_counts[code] = self.halt_counts.get(code, 0) + 1
            # 只记录状态进入事件；页面约 5 Hz，仍能看到两次刷新之间发生的短暂失效。
            event = dict(t=stamp, state=state, code=code,
                         odom_age=None if stamp is None or pose_stamp is None else stamp-pose_stamp,
                         depth_age=None if stamp is None or self.grid.last_depth is None else stamp-self.grid.last_depth,
                         controller_age=None if stamp is None or not math.isfinite(self.external_stamp) else stamp-self.external_stamp)
            self.halt_events = (self.halt_events + [event])[-32:]
        self.code, self.safety_path = code, []
        if state in ('PAUSED','ESTOP','INITIALIZING','TARGET_LOST','ODOM_LOST','INPUT_LOST') or code == 'MAP_STALE':
            # 临时出口请求不能跨越输入失效；恢复后基于新地图和新候选重新判定。
            self.continuation_limited = False
            self.stationary_since,self.face_active = None,False
            self.tracking_requested = False
            if self.plan.kind == 'OBSERVING' or self.observation.plan is not None or self.use_follow_intent:
                self.invalidate_plan(code,now)
                if self.use_follow_intent:
                    # 同一控制时刻提交又被中断的旧搜索也必须拒收，不能仅靠普通结果年龄门槛。
                    stamp = self.last_tick if now is None else now
                    if stamp is not None:
                        self.minimum_plan_stamp = max(self.minimum_plan_stamp, math.nextafter(stamp, math.inf))
            if self.use_follow_intent:
                self.intent_reference = None
                self.intent_diagnostic = dict(enabled=True, status=code)
                if state == 'TARGET_LOST':
                    self.follow_intent.update(None, None, valid=False)
                elif state in ('PAUSED', 'ESTOP') and self.state != state:
                    # 明确暂停可重新建立起始意图，恢复时不要求追赶已经被有限历史删除的转角。
                    self.follow_intent.reset()
                    self.intent_tracker.reset()
        return self.stop(state, reason)

    def invalidate_plan(self, reason, now=None):
        """同时撤销观察任务、执行路径及旧候选，恢复输入后不能继续无上下文的观察移动。"""
        self.observation.clear()
        self.plan = Plan(reason=reason)
        self.plan_revision += 1
        self.external_active,self.external_stamp = False,-math.inf
        self.last_plan = -math.inf
        stamp = self.last_tick if now is None else now
        if stamp is not None:
            self.minimum_plan_stamp = max(self.minimum_plan_stamp,stamp)
        # 正在搜索的旧任务可能无法取消；返回时仍按上述时间边界丢弃。
        if self.future is not None and self.future.cancel():
            self.future = None

    def compute_plan(self, snapshot, pose, target, velocity, spacing, requested, tried, intent_reference=None):
        """保留同步三元组测试入口；正式异步提交使用顶层函数，绝不序列化 self。"""
        return search_plan(snapshot, pose, target, velocity, spacing, requested, tried,
                           intent_reference, self.use_follow_intent,
                           copy.deepcopy(self.planner.previous), self.planner.side)

    def update_plan(self, pose, target, velocity, spacing, now, parking_safe=True, intent_reference=None):
        """只允许一个搜索，过旧结果丢弃；新地图仍需重新验证执行路径。"""
        outcome = self.observation.advance(pose,now,self.grid,parking_safe=parking_safe)
        if outcome in ('OBSERVATION_ORIENT','OBSERVATION_REGION_LEFT'):
            self.plan = copy.deepcopy(self.observation.plan)
            self.plan_revision += 1
            # 旧候选来自移动任务，不能穿过阶段切换继续驱动固定位置的观察动作。
            self.external_active,self.external_stamp = False,-math.inf
        elif outcome is not None and self.observation.plan is None:
            self.invalidate_plan(outcome,now)
        if self.future is not None and self.future.done():
            completed, self.future = self.future, None
            try:
                plan, self.plan_ms, stamp = completed.result()
            except Exception as error:
                # 进程死亡、序列化失败等均撤销候选；损坏的池不能反复重试或重放旧动作。
                self.search_failed = True
                self.search_error = f'{type(error).__name__}: {error}'
                self.rejected_plan_count += 1
                self.rejected_plan_reason = 'SEARCH_WORKER_FAILED'
                self.invalidate_plan('SEARCH_WORKER_FAILED', now)
                self.tracking_requested = False
                self.halt('WAITING', 'SEARCH_WORKER_FAILED', '导航搜索工作任务异常', now=now)
                return
            if self.use_process_search and plan.search_pid:
                self.search_worker_pids.add(plan.search_pid)
            self.plan_cpu_ms = plan.search_cpu_ms
            if self.minimum_plan_stamp <= stamp <= now and now-stamp < .8:
                free,allowed,_ = self.grid.layers(now)
                active = self.observation.plan is not None and bool(self.path_ahead(pose,free))
                # 搜索读取的是过去的快照，返回时先校验当前连接和整条路线，再提交任务。
                # 先 begin 再发现 PATH_INVALID，会清掉有效旧任务并反复创建无效观察动作。
                candidate_valid = not plan.path or bool(self.path_ahead(pose,free,plan))
                if not candidate_valid:
                    self.rejected_plan_count += 1
                    self.rejected_plan_reason = 'RETURNED_PATH_INVALID'
                # 优先接入已打开的跟随通道；观察姿态只有任务完成、明确失败或地图冲突才替换。
                if candidate_valid and (plan.kind == 'FOLLOWING' or not active):
                    excluded = (plan.kind == 'OBSERVING' and plan.path
                                and self.observation.excluded(plan.path[-1],plan.look_yaw,self.grid.resolution))
                    if not excluded:
                        self.observation.clear()
                        if plan.kind == 'OBSERVING':
                            self.observation.begin(plan,pose,now,self.grid)
                        self.plan,self.plan_revision = copy.deepcopy(plan),self.plan_revision+1
                        # 只有真正接入的路径才影响后续终点和换边，过期/非法候选不污染状态。
                        self.planner.previous, self.planner.side = copy.deepcopy(plan), plan.search_side
            else:
                self.rejected_plan_count += 1
                self.rejected_plan_reason = 'SEARCH_RESULT_STALE'
        if not self.search_closed and not self.search_failed and self.future is None and now-self.last_plan >= PLAN_UPDATE_SECONDS:
            self.last_plan = now
            try:
                self.future = self.pool.submit(search_plan, copy.deepcopy(self.grid), copy.copy(pose),
                                               target.copy(), list(velocity), spacing, now,copy.deepcopy(self.observation.tried),
                                               intent_reference, self.use_follow_intent,
                                               copy.deepcopy(self.planner.previous), self.planner.side,
                                               self.continuation_limited and abs(self.motion[0])<.12)
            except Exception as error:
                # 提交阶段也可能因进程池损坏而失败；输出与任务上下文必须同样撤销。
                self.search_failed = True
                self.search_error = f'{type(error).__name__}: {error}'
                self.rejected_plan_count += 1
                self.rejected_plan_reason = 'SEARCH_WORKER_FAILED'
                self.invalidate_plan('SEARCH_WORKER_FAILED', now)
                self.tracking_requested = False
                self.halt('WAITING', 'SEARCH_WORKER_FAILED', '导航搜索任务无法提交', now=now)

    def path_ahead(self, pose, free, plan=None):
        """裁去已经过的路径，连接前方路径，禁止回头追旧起点或跨障碍抄近路。"""
        plan = self.plan if plan is None else plan
        if not plan.path:
            return []
        dense = [plan.path[0]]
        for a,b in zip(plan.path,plan.path[1:]):
            dense.extend(np.linspace(a,b,max(2,int(math.dist(a,b)/.08)+1))[1:].tolist())
        here = [pose.x,pose.y]
        nearest = min(range(len(dense)),key=lambda i: math.dist(here,dense[i]))
        # 路径重规划有延迟；只接回前方0.24m会把小横向误差变成急转。
        # 随实测速度选择约一个转向响应周期的前向连接，逐段校验仍拒绝跨墙抄近路。
        join_distance = .5 + .875 * min(MAX_NAVIGATION_SPEED, max(0., self.motion[0]))
        index = nearest
        supplied = 0.
        while index+1 < len(dense) and supplied < join_distance:
            supplied += math.dist(dense[index],dense[index+1])
            index += 1
        while index > nearest and not self.native_braking.route(self.grid,free,[here,dense[index]],pose.yaw):
            index -= 1
        if not self.native_braking.route(self.grid,free,[here,dense[index]],pose.yaw):
            return []
        route = [here]+dense[index:]
        final_yaw = plan.look_yaw if plan.kind in ('OBSERVING','FACING') else None
        if not self.native_braking.route(self.grid,free,route,pose.yaw,final_yaw):
            return []
        return route

    def braking_safe(self, pose, command, allowed, actual=False):
        """检查三类残余响应与停车包络；正向跟随还须保留名义停车末端的已知出口。"""
        v,w = command
        if actual:
            result = self.braking_trajectory(pose,v,w,self.motion[1],allowed)
            self.guard_diagnostic = dict(t=pose.t,kind='actual_motion',safe=result[0],
                                         check=dict(self.trajectory_diagnostic))
            return result
        if abs(v)+abs(w) < 1e-9:
            # 全零是制动请求，机身仍会移动；原来只预测零轨迹会掩盖残余运动。
            return self.braking_trajectory(pose,self.motion[0],self.motion[2],self.motion[1],allowed)
        forward = max(v,max(0.,self.motion[0]))
        turns = dict.fromkeys((w,self.motion[2],.5*(w+self.motion[2])))
        primary = []
        for turn in turns:
            safe,route = self.braking_trajectory(pose,forward,turn,self.motion[1],allowed)
            if not safe:
                return False,route
            if not primary:
                primary = route
                if self.plan.kind=='FOLLOWING' and v>1e-9:
                    # 加权后的MPPI输出及舒适平滑都可能破坏候选的续行条件，须在执行前重查。
                    # 零命令仍允许实际制动；观察用途按原区域验证，不套用这个跟随出口条件。
                    end=self.trajectory_diagnostic.get('stop_pose')
                    if end is None or not self.native_braking.continuable(self.grid,allowed,Pose(pose.t,*end)):
                        self.trajectory_diagnostic.update(safe=False,rejection='CONTINUATION_UNCONFIRMED')
                        return False,route
        # 三种组合补上已知响应失配，不冒充覆盖全部过冲/时延的实机可达轨迹管。
        return True,primary

    def braking_trajectory(self, pose, v, w, lateral, allowed):
        """检查一组前速、转速与侧移的保持/制动过程，包含每步之间的全部触碰格。"""
        x,y,yaw = pose.x,pose.y,pose.yaw
        route = [[x,y]]
        previous = [x,y,yaw]
        stamp = pose.t if self.grid.last_depth is None else self.grid.last_depth
        latency = execution_hold_seconds(pose.t-stamp,self.grid.static_history)
        native = self.native_braking.brake(self.grid,allowed,pose,v,w,lateral,latency,BRAKE_DECELERATION)
        if native is not None:
            safe,route,self.trajectory_diagnostic = native
            return safe,route
        duration = latency+max(abs(v)/BRAKE_DECELERATION,abs(w),abs(lateral)/BRAKE_DECELERATION)
        self.trajectory_diagnostic = dict(forward=v,turn=w,lateral=lateral,hold_s=latency,
                                          duration_s=duration,safe=True)
        for index in range(max(1,math.ceil(duration/.04))):
            if index*.04 >= latency:
                v = math.copysign(max(0,abs(v)-BRAKE_DECELERATION*.04),v)
                lateral = math.copysign(max(0,abs(lateral)-BRAKE_DECELERATION*.04),lateral)
                w = math.copysign(max(0,abs(w)-.04),w)
            x += (v*math.cos(yaw)-lateral*math.sin(yaw))*.04
            y += (v*math.sin(yaw)+lateral*math.cos(yaw))*.04
            yaw += w*.04
            if not self.grid.motion_clear(allowed,previous,[x,y,yaw]):
                self.trajectory_diagnostic.update(safe=False,first_rejected_segment=[route[-1],[x,y]],
                                                  elapsed_s=(index+1)*.04)
                return False,route
            route.append([x,y])
            previous = [x,y,yaw]
        self.trajectory_diagnostic['stop_pose'] = [x,y,yaw]
        return True,route

    def constrain_command(self, pose, original, allowed):
        """前进受限时保留仍安全的转向，每个候选重复校验完整停车过程。"""
        candidates = [(original[0]*scale,original[1]) for scale in (1.,.75,.5,.25,0.)]
        candidates += [(0.,original[1]*.5),(0.,0.)]
        rejected = []
        self.continuation_limited = False
        for command in candidates:
            safe,route = self.braking_safe(pose,command,allowed)
            if safe:
                if command[0]>1e-9:
                    self.continuation_limited = False
                # 只保存本拍的候选拒收证据；诊断不反向改变碰撞边界或放宽门槛。
                self.guard_diagnostic = dict(t=pose.t,kind='candidate',requested=list(original),
                                             accepted=list(command),rejected=rejected,safe=True)
                return command,route
            rejected.append(dict(command=list(command),check=dict(self.trajectory_diagnostic)))
            if self.trajectory_diagnostic.get('rejection')=='CONTINUATION_UNCONFIRMED':
                self.continuation_limited = True
        self.guard_diagnostic = dict(t=pose.t,kind='candidate',requested=list(original),
                                     accepted=[0.,0.],rejected=rejected,safe=False)
        return (0.,0.),[]

    def prepare_facing(self, pose, target, free, now):
        """人稳定停下后在已知净空内看向人；完整旋转不安全时继续保持原地。"""
        if self.stationary_since is None:
            self.stationary_since = now
        bearing = math.atan2(target[1]-pose.y,target[0]-pose.x)
        error = abs(wrap(bearing-pose.yaw))
        if now-self.stationary_since < FACE_STATIONARY_SECONDS:
            self.face_diagnostic = dict(status='WAIT_STABLE',error=error)
            return False
        threshold = FACE_EXIT_ANGLE if self.face_active else FACE_ENTER_ANGLE
        if error<=threshold:
            self.face_active = False
            self.face_diagnostic = dict(status='ALIGNED',error=error)
            return False
        if not self.grid.motion_clear(free,[pose.x,pose.y,pose.yaw],[pose.x,pose.y,bearing]):
            self.face_active = False
            self.face_diagnostic = dict(status='ROTATION_BLOCKED',error=error)
            return False
        # 看向任务不依赖观察收益，也不能借人方位抢占正在绕障的路径。
        if self.plan.kind != 'FACING':
            self.invalidate_plan('FOLLOW_FACE_TARGET',now)
            self.face_started,self.face_progress,self.face_best = now,now,error
        elif error<self.face_best-.02:
            self.face_progress,self.face_best = now,error
        if now-self.face_progress>6. or now-self.face_started>15.:
            self.face_active = False
            self.face_diagnostic = dict(status='TURN_STALLED',error=error)
            return False
        changed = self.plan.kind!='FACING' or abs(wrap(bearing-self.plan.look_yaw))>.04
        self.plan = Plan('FACING','FOLLOW_FACE_TARGET',path=[[pose.x,pose.y]],look_yaw=bearing,target=list(target))
        if changed:
            self.plan_revision += 1
        self.face_active = True
        self.face_diagnostic = dict(status='TURNING',error=error,goal_yaw=bearing)
        return True

    def step(self, now, enabled=True, signal_valid=True, ready=True, emergency=False):
        """检查输入和通道，连续限速，再验证 MPPI 的真实待执行动作。"""
        self.tracking_requested = False
        dt = .02 if self.last_tick is None else clamp(now-self.last_tick,.001,.10)
        self.last_tick = now
        self.control_pose = self.history.current(now)
        if emergency:
            return self.halt('ESTOP','ESTOP','用户急停')
        if not enabled:
            self.continuation_limited = False
            self.stall_since,self.progress_pose = None,None
            self.progress_failed,self.observing_since = False,None
            return self.halt('PAUSED','PAUSED','跟随已暂停')
        if not ready:
            return self.halt('INITIALIZING','NOT_READY','姿态准备中或异常')
        if not signal_valid or self.target_stamp is None or not 0 <= now-self.target_stamp <= self.target_timeout:
            return self.halt('TARGET_LOST','TARGET_INVALID','UWB 数据无效或过期')
        if self.control_pose is None:
            return self.halt('ODOM_LOST','ODOM_STALE','里程计数据过期')
        pose = self.control_pose
        if pose.motion is not None:
            self.motion = pose.motion
        self.grid.recenter(pose.x,pose.y)
        if not self.grid.confirmed:
            return self.halt('WAITING','START_UNCONFIRMED','请先暂停并确认起始周围 1.2 m 净空')
        if self.grid.last_depth is None or not 0 <= now-self.grid.last_depth <= self.depth_timeout:
            return self.halt('WAITING','MAP_STALE','前视深度缺失或过期')
        free,allowed,_ = self.grid.layers(now)
        self.footprint = self.grid.footprint_status(pose.x,pose.y,now,pose.yaw)
        if self.footprint['code'] != 'CLEAR':
            return self.halt('WAITING',self.footprint['code'],'机身包络净空不足，查看阻塞栅格诊断')
        velocity = np.asarray(self.target_velocity)
        target = np.asarray(self.target)+velocity*min(now-self.target_stamp,.2)
        distance = float(np.linalg.norm(target-[pose.x,pose.y]))
        self.last_distance = distance
        self.heading_diagnostic = heading_diagnostics(pose,target,self.plan.path,velocity,
                                                      self.command,self.motion,self.target_stamp,self.last_world_measurement)
        if distance < .85:
            return self.halt('TOO_CLOSE','TARGET_TOO_CLOSE','目标进入近距离保护区')
        speed = float(np.linalg.norm(velocity))
        spacing = self.desired_distance+.3*min(speed,.7)
        arrival_distance = spacing+FOLLOW_REGION_HALF_WIDTH+FOLLOW_GOAL_TOLERANCE
        # 静止保持也是停车请求，先核验当前残余运动，不能靠“已到跟随距离”跳过制动诊断。
        actual_safe,_ = self.braking_safe(pose,(self.motion[0],self.motion[2]),free,actual=True)
        if not actual_safe:
            # 执行停车与撤销优化是两个决策。输入仍有效、旧跟随路径仍几何可行时，
            # 保留MPPI热启动并送零速度参考，避免每次制动后等待新动作的首个输出。
            # 本拍以及制动条件未恢复的后续各拍仍只输出零，候选绝不绕过最终保护。
            route = (self.path_ahead(pose, free) if self.plan.kind == 'FOLLOWING'
                     and not self.progress_failed and not self.search_failed else [])
            if route:
                self.tracking_requested = True
                self.speed_reference = dict(speed=0., reason='BRAKING_EXECUTION_STOP',
                                            route_length_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])))
            return self.halt('WAITING','BRAKING_SPACE','实测运动的停车空间不足')
        sight = self.grid.segment_cells([pose.x,pose.y],target)
        occupied_sight = sight is not None and any(self.grid.occupied[cell] for cell in sight)
        facing = False
        if distance <= arrival_distance and speed < .08 and not occupied_sight:
            facing = self.prepare_facing(pose,target,free,now)
            # 人已停在距离带内、狗能安全停下，缺少人与狗之间的部分视线证据不应迫使狗绕圈前进。
            # 未知视线只允许保持当前位置；明确的隔墙证据仍交给绕行规划，不能称为跟随到位。
            self.stall_since,self.progress_pose = None,None
            self.observing_since = None
            if not facing:
                self.tracking_requested,self.external_active = False,False
                verified = self.grid.segment_clear(free,[pose.x,pose.y],target)
                return self.halt('HOLDING','FOLLOW_DISTANCE' if verified else 'FOLLOW_DISTANCE_UNOBSERVED',
                                 '目标停下，保持舒适间距' if verified else '目标停下，保持当前位置；视线证据不完整')
        else:
            self.stationary_since,self.face_active = None,False
            self.face_diagnostic = dict(status='MOVING_PATH_PRIORITY')
            if self.plan.kind == 'FACING':
                self.invalidate_plan('FOLLOW_RESUMED',now)
        if self.progress_failed:
            return self.halt('WAITING','NO_PROGRESS','本轮探索无进展，请暂停后再恢复以重新尝试')
        if self.plan.kind == 'OBSERVING' and self.observation.plan is None:
            # 观察的路径用途依赖区域和阶段元数据，任务被清空后旧路径必须同步失效。
            self.invalidate_plan('OBSERVATION_CONTEXT_LOST',now)
        # 有限前视避免把移动跟随区域当成一串停车终点；目标意图不作为自由空间证据。
        reference = None
        if self.use_follow_intent:
            reference = self.prepare_intent_reference(now,pose,spacing)
            if reference is None or reference.status != 'FRESH':
                code = 'FOLLOW_TRAIL_INVALID' if reference is None else 'FOLLOW_TRAIL_HISTORY_LOST'
                self.invalidate_plan(code,now)
                return self.halt('WAITING',code,'目标轨迹意图失效，请暂停后重新建立跟随意图')
        if not facing:
            # 连续跟随参考覆盖MPPI预测窗和一次重规划间隔；1.2s短参考会让2.4s优化器提前刹停。
            # 这是目标意图外推，不是自由空间证据：搜索及执行仍只走相机确认的通道。
            horizon = MPPI_HORIZON_SECONDS+PLAN_UPDATE_SECONDS
            self.update_plan(pose,target+velocity*horizon,velocity,spacing,now,parking_safe=actual_safe,
                             intent_reference=reference)
        if not self.plan.path:
            return self.halt('WAITING',self.plan.reason if self.future is None else 'PLAN_PENDING','等待可执行通道')
        route = self.path_ahead(pose,free)
        if not route:
            self.invalidate_plan('PATH_INVALID',now)
            return self.halt('WAITING','PATH_INVALID','路径与最新地图冲突，重新搜索')
        if self.plan.kind == 'OBSERVING':
            if self.observing_since is None:
                self.observing_since = now
            elif now-self.observing_since > 40.:
                # 连续观察必须有总预算；大量与通行无关的新自由格不能无限延长墙前探索。
                self.progress_failed = True
                return self.halt('WAITING','NO_PROGRESS','观察预算已用尽，请暂停后再恢复以重新尝试')
        else:
            self.observing_since = None
        if self.observation.stage == 'EVALUATE':
            return self.halt('OBSERVING','OBSERVATION_EVALUATING','等待当前朝向的新深度，确认通道')
        if self.progress_pose is None or math.dist(self.progress_pose[:2],[pose.x,pose.y]) > .2 or int(allowed.sum()) > self.progress_pose[2]+25:
            self.progress_pose,self.stall_since = [pose.x,pose.y,int(allowed.sum())],now
        if self.stall_since is not None and now-self.stall_since > 15:
            # 失败保持到明确的暂停/恢复；后续地图计数抖动不能自行重新启动。
            self.progress_failed = True
            return self.halt('WAITING','NO_PROGRESS','观察或绕行无进展，暂停后可重新尝试')
        self.tracking_requested = True
        self.phase = ('FACING' if facing else 'OBSERVING' if self.plan.kind == 'OBSERVING' else
                      'DETOUR' if len(self.plan.path)>2 else 'FOLLOWING')
        self.speed_reference = route_speed_reference(route, pose.yaw, speed, distance, spacing,
                                                     now-self.grid.last_depth, self.plan.kind,
                                                     orient=facing or self.observation.stage == 'ORIENT',
                                                     hold_seconds=execution_hold_seconds(now-self.grid.last_depth,
                                                                                        self.grid.static_history))
        if not self.external_active or not 0 <= now-self.external_stamp <= .3:
            return self.halt('WAITING','CONTROLLER_STALE','等待有效 MPPI 轨迹输出')
        forward,turn = self.external_command
        if not all(math.isfinite(v) for v in (forward,turn)):
            return self.halt('WAITING','CONTROLLER_INVALID','MPPI 返回非有限命令')
        if self.observation.stage == 'ORIENT' or facing:
            # 观察位置已到达；MPPI 的数值微小前进量不应在反复看向时累计成靠边漂移。
            forward = 0.
        # 距离需求和曲率共同限速，宽阔绕行不再固定降至 0.22 m/s。
        cap = min(MAX_NAVIGATION_SPEED,self.speed_reference['speed'])
        curvature = abs(turn)/max(abs(forward),.12)
        cap = min(cap,math.sqrt(.45/max(curvature,.01)))
        if self.phase == 'OBSERVING':
            cap = min(cap,.45)
        if facing:
            cap = 0.
        self.speed_limit = cap
        desired = (clamp(forward,0.,cap),clamp(turn,-MAX_NAVIGATION_TURN,MAX_NAVIGATION_TURN))
        reset_axes=[]
        for i,(value,accel,jerk) in enumerate(zip(desired,(.6,1.6),(2.,4.))):
            lower,upper=(0.,cap) if i==0 else (-MAX_NAVIGATION_TURN,MAX_NAVIGATION_TURN)
            self.command[i],self.acceleration[i],reset=smooth_axis(
                self.command[i],self.acceleration[i],value,dt,accel,jerk,lower,upper)
            if reset:
                reset_axes.append(i)
        self.smoothing_diagnostic=dict(desired=list(desired),reversed_acceleration_axes=reset_axes,
                                       acceleration=list(self.acceleration))
        original = tuple(self.command)
        command,self.safety_path = self.constrain_command(pose,original,free)
        self.safety_limited = command != original
        for i in range(2):
            if command[i] != original[i]:
                self.acceleration[i] = 0.
        self.command = list(command)
        self.heading_diagnostic = heading_diagnostics(pose,target,route,velocity,command,self.motion,
                                                      self.target_stamp,self.last_world_measurement)
        self.state,self.code = self.phase,'EXECUTION_LIMITED' if self.safety_limited else self.plan.reason
        self.reason = 'MPPI 连续跟踪；停车空间限速' if self.safety_limited else 'MPPI 沿已观测通道连续跟随'
        return command

    def diagnostics(self, now):
        """记录路径、原始 MPPI 命令和最终命令之间的约束原因。"""
        return dict(code=self.code,phase=self.phase,path=self.plan.path,plan_kind=self.plan.kind,
                    look_yaw=self.plan.look_yaw,safety_path=self.safety_path,plan_ms=self.plan_ms,
                    plan_cpu_ms=self.plan_cpu_ms,
                    search_execution_mode='process' if self.use_process_search else 'thread',
                    search_worker_pids=sorted(self.search_worker_pids),
                    search_failed=self.search_failed,search_error=self.search_error,
                    map=self.grid.display(now),tracker='nav2_mppi',motion=list(self.motion),
                    odometry_alignment=dict(control_t=now,
                        selected_stamp=None if self.control_pose is None else self.control_pose.t,
                        latest_received_stamp=None if not self.history.values else self.history.values[-1].t,
                        policy='latest_measurement_at_or_before_control_clock'),
                    speed_reference=dict(self.speed_reference),
                    raw_command=list(self.external_command),speed_limit=self.speed_limit,
                    footprint=self.footprint,plan_revision=self.plan_revision,
                    search_pending=self.future is not None,controller_active=self.external_active,
                    controller_age=None if not math.isfinite(self.external_stamp) else now-self.external_stamp,
                    rejected_plan_count=self.rejected_plan_count,rejected_plan_reason=self.rejected_plan_reason,
                    execution_guard=self.guard_diagnostic,
                    smoothing=self.smoothing_diagnostic,
                    continuation_limited=self.continuation_limited,
                    halt_counts=dict(self.halt_counts),halt_events=list(self.halt_events),
                    command_limits=dict(forward_mps=MAX_NAVIGATION_SPEED,angular_radps=MAX_NAVIGATION_TURN),
                    heading=self.heading_diagnostic,facing=self.face_diagnostic,
                    follow_intent=self.intent_diagnostic,
                    observation=self.observation.diagnostics())
