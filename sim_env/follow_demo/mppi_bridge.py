"""将局部通道和历史几何送入标准 Nav2；拒绝跨重置或取消后的旧命令。"""
import math
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath
from std_msgs.msg import Float64MultiArray
from .navigation_config import BRAKE_DECELERATION, execution_hold_seconds


class MppiBridge:
    """使用标准 FollowPath 动作更新移动参考，最终 /cmd_vel 仍只由跟随节点发布。"""
    def __init__(self,node):
        """建立专属命名空间连接，地图只含相机证据，不读取仿真场景。"""
        self.node = node
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = node.create_publisher(OccupancyGrid,'/follow_demo/local_geometry',qos)
        self.path_pub = node.create_publisher(Path,'/follow_demo/reference_path',1)
        self.speed_pub = node.create_publisher(TwistStamped,'/follow_demo/reference_speed',1)
        self.braking_pub = node.create_publisher(Float64MultiArray,'/follow_demo/braking_limits',1)
        self.client = ActionClient(node,FollowPath,'/go2_follow_mppi/follow_path')
        self.subscription = node.create_subscription(Twist,'/follow_demo/mppi_cmd',self.on_command,1)
        self.generation,self.sequence = 0,0
        self.handle,self.pending = None,None
        self.last_map,self.last_sent,self.revision = -math.inf,-math.inf,-1
        self.accepted_at,self.wanted = math.inf,False
        self.error = ''
        self.controller,self.last_result = '',None

    def reset(self):
        """使所有旧动作回调失效，取消仍在运行的动作；旧速度不能穿过场景重置。"""
        self.generation += 1
        self.wanted = False
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.handle,self.pending = None,None
        self.revision,self.last_map,self.last_sent = -1,-math.inf,-math.inf
        self.node.core.external_active = False
        self.node.core.external_stamp = -math.inf

    def on_command(self,message):
        """只有当前动作已被接受且仍需要跟踪时，才保存新候选速度。"""
        now = self.node.get_clock().now().nanoseconds*1e-9
        if self.wanted and self.handle is not None and now >= self.accepted_at:
            self.node.core.external_command = (message.linear.x,message.angular.z)
            self.node.core.external_stamp = now
            self.node.core.external_active = True

    def update(self,now):
        """发布几何及优化参考；有效路径的临时制动保留优化，失效和暂停撤销动作。"""
        core = self.node.core
        reference = TwistStamped()
        reference.header.stamp = self.node.get_clock().now().to_msg()
        reference.header.frame_id = 'base_footprint'
        reference.twist.linear.x = float(core.speed_reference['speed']) if core.tracking_requested else 0.
        self.speed_pub.publish(reference)
        # 协议顺序：采样时刻s、保持时间s、线制动m/s²、角制动rad/s²；与末级保护同口径。
        stamp = core.grid.last_depth
        hold = execution_hold_seconds(now-stamp,core.grid.static_history) if stamp is not None else .75
        self.braking_pub.publish(Float64MultiArray(data=[float(now),float(hold),BRAKE_DECELERATION,1.]))
        if now-self.last_map >= .15:
            self.last_map = now
            grid = core.grid
            free,_,_ = grid.layers(now)
            message = OccupancyGrid()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = 'odom'
            message.info.resolution,message.info.width,message.info.height = grid.resolution,grid.size,grid.size
            message.info.origin.position.x,message.info.origin.position.y = grid.origin.tolist()
            message.info.origin.orientation.w = 1.
            # 对 MPPI 也把未知当障碍，禁止优化器从墙后或未观察区域抄近路。
            message.data = np.where(free,0,100).astype(np.int8).ravel().tolist()
            self.map_pub.publish(message)
        if not core.tracking_requested:
            if self.wanted:
                self.reset()
            return
        facing = core.plan.kind == 'FACING'
        orient = facing or (core.plan.kind == 'OBSERVING' and core.observation.stage == 'ORIENT')
        controller = 'Observe' if orient else 'FollowPath'
        if self.wanted and self.controller != controller:
            # 控制用途变化时撤销旧动作及其回调；普通连续路径更新仍保留 MPPI 热启动。
            # 新动作接受前 on_command 不接收旧移动候选，避免转向任务继续前进。
            self.reset()
        self.wanted = True
        if not self.client.server_is_ready() or self.pending is not None:
            return
        if core.plan_revision == self.revision and self.handle is not None:
            return
        if now-self.last_sent < .3 or not core.history.values:
            return
        free,_,_ = core.grid.layers(now)
        # 路径接入与末级保护使用同一控制位姿，不混入暂时领先/clock的里程计样本。
        pose = core.control_pose
        if pose is None:
            return
        route = core.path_ahead(pose,free)
        if not route:
            return
        self.last_sent,self.revision = now,core.plan_revision
        goal = FollowPath.Goal()
        # 接近观察位置时只沿路径行驶，禁止终点朝向代价提前把机器人转离尚未走完的路线。
        # 到达后位置锚定为实测姿态，此时才让 MPPI 完成看向动作。
        goal.controller_id = controller
        goal.goal_checker_id = 'face_goal' if facing else 'observe_goal' if orient else 'follow_goal'
        self.controller = goal.controller_id
        goal.path.header.stamp,goal.path.header.frame_id = self.node.get_clock().now().to_msg(),'odom'
        for index,point in enumerate(route):
            item = PoseStamped()
            item.header = goal.path.header
            item.pose.position.x,item.pose.position.y = [float(v) for v in point]
            if index+1 < len(route):
                next_point = route[index+1]
                yaw = math.atan2(next_point[1]-point[1],next_point[0]-point[0])
            else:
                if orient:
                    yaw = core.plan.look_yaw
                elif index:
                    previous_point = route[index-1]
                    yaw = math.atan2(point[1]-previous_point[1],point[0]-previous_point[0])
                else:
                    yaw = pose.yaw
            item.pose.orientation.z,item.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            goal.path.poses.append(item)
        self.path_pub.publish(goal.path)
        self.sequence += 1
        generation,sequence = self.generation,self.sequence
        self.pending = self.client.send_goal_async(goal)
        self.pending.add_done_callback(lambda future:self.accepted(future,generation,sequence))

    def accepted(self,future,generation,sequence):
        """只处理本轮动作接受结果；过时的已接受动作立即取消。"""
        handle = future.result()
        if generation != self.generation or sequence != self.sequence:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.pending = None
        if not handle.accepted:
            self.error,self.handle = 'MPPI 动作未接受',None
            return
        self.handle,self.error = handle,''
        self.accepted_at = self.node.get_clock().now().nanoseconds*1e-9
        handle.get_result_async().add_done_callback(lambda result:self.finished(result,generation,sequence))

    def finished(self,future,generation,sequence):
        """动作结束后等待新的路径和速度，不能继续发送上一动作的最后命令。"""
        if generation != self.generation or sequence != self.sequence:
            return
        result = future.result()
        self.handle = None
        self.last_result = dict(status=result.status,t=self.node.get_clock().now().nanoseconds*1e-9,
                                error_code=getattr(result.result,'error_code',None),
                                error_msg=getattr(result.result,'error_msg',None))
        self.node.core.external_active = False
        self.error = '' if result.status == 4 else f'MPPI 动作结束状态 {result.status}'

    def diagnostics(self):
        """区分动作接受、等待、成功结束和控制输出过期，避免统一归因于控制器失联。"""
        return dict(controller=self.controller,sequence=self.sequence,generation=self.generation,
                    pending=self.pending is not None,accepted=self.handle is not None,
                    wanted=self.wanted,last_result=self.last_result)
