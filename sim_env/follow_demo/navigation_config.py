"""跨搜索、控制与终点判断共享的仿真配置，不代表实机标定参数。"""

# 搜索的舒适距离带与 MPPI 位置容差必须一起用于上层到达判定。
# 否则控制器已到终点，上层仍催促前进，会不断重发厘米级路径。
FOLLOW_REGION_HALF_WIDTH = .22
FOLLOW_GOAL_TOLERANCE = .08
# 连续跟随参考尽量覆盖一个 MPPI 预测窗口及下一次搜索间隔；只是软偏好，不封死短通道。
MPPI_HORIZON_SECONDS = 2.4
PLAN_UPDATE_SECONDS = .4
# 姿态搜索计入转向代价后，提高供给参考的软偏好，防止移动目标的距离带包含原地点时原地等待。
FOLLOW_REFERENCE_SHORTFALL_COST = 2.5
# 交错障碍中欧氏间距不大，但目标沿折线路径的领先可超过12 m。
# 保留32 m（目标0.8时约40秒）意图，512点覆盖0.08 m采样预算；不扩大地图或自由证据。
FOLLOW_HISTORY_LENGTH = 32.
FOLLOW_HISTORY_POINTS = 512
# 该距离只用于参考点接近及无收益终点的失败判定；观察准入由实际位置的区域与视野验证决定。
OBSERVATION_POSITION_TOLERANCE = FOLLOW_GOAL_TOLERANCE
OBSERVATION_YAW_TOLERANCE = .20
# 用户要求的命令边界；实际步态模型可达转速另行测量，不能将命令上限当成实测能力。
MAX_NAVIGATION_SPEED = .8
MAX_NAVIGATION_TURN = 1.
# 路径速度参考和最终停车包络共享保持/制动参数，避免各层假设不同执行能力。
BRAKE_DECELERATION = .45
CONTROL_HOLD_SECONDS = .25


def execution_hold_seconds(depth_age, static_history):
    """区分执行延迟和观测年龄；静态地图已按采集时刻TF补偿，不能再次计入同一年龄。"""
    # 静态场景的历史自由证据持续有效，未知仍禁行；输入新鲜度由独立健康门槛检查。
    # 非静态模式仍保留观测年龄增量，该模式不能据本轮静态验收宣称动态障碍安全。
    return CONTROL_HOLD_SECONDS + (0. if static_history else min(.5,max(0.,depth_age)))
# base_footprint 中心矩形；不再使用圆形硬膨胀判断机器人能否通过。
ROBOT_LENGTH = .70
ROBOT_WIDTH = .32
# 行走方向的软代价不触发原地旋转状态，减小原29°方向代价门槛。
PATH_ANGLE_SOFT_THRESHOLD = .15
# 停人后的朝向使用滞回及稳定时间，防止测角噪声引起反复转动。
FACE_ENTER_ANGLE = .12
FACE_EXIT_ANGLE = .06
FACE_STATIONARY_SECONDS = .4
