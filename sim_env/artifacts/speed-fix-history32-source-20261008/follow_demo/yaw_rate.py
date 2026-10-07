"""独立角速度跟踪实验：不改变线速度，不保证未经测试的转速可达。"""
import math


class YawRateServo:
    """用有来源的前馈及实测角速度误差反馈生成有限 ONNX 输入。"""

    def __init__(self, kp=.35, ki=.5, filter_tau=.08):
        """配置实验参数；前馈端点来自锁定模型基准，反馈增益仍须物理验收。"""
        self.kp, self.ki, self.filter_tau = float(kp), float(ki), float(filter_tau)
        # heading 的未饱和局部模型等价于 Ki=.5 的角速度误差积分。
        # 新增比例项 .35 只做温和阻尼；它不是通过模型辨识得到的最优增益。
        # .08秒低通削弱步态角速度波动，该时间常数也是待验收的实验选择。
        self.reset()

    def reset(self):
        """复位积分、过滤与诊断，暂停或全零命令不保留旧转向误差。"""
        self.integral = 0.
        self.filtered_rate = None
        self.last = dict(requested_rate=0., measured_rate=0., filtered_rate=0.,
                         feedforward=0., error=0., integral=0., output=0., saturated=False)

    @staticmethod
    def feedforward_gain(forward):
        """在两项实测端点间插值；其他前速段仅是明确的实验假设。"""
        # 原地：稳态模型输入 .958 / 实际转速 .399 ≈ 2.4。
        # 前进.5：稳态模型输入 .467 / 实际转速 .294 ≈ 1.6。
        # 来源：execution-region-20261004(-samples).json 的 heading 配对基准。
        # 不加入最低角速度；低前速的非线性响应没有由此获得认证。
        phase = min(1., max(0., abs(float(forward))/.5))
        return 2.4-.8*phase

    def update(self, forward, requested, measured, dt=.02):
        """输出前馈+PI并限幅±1，冻结推动饱和的积分，反向误差仍可释放积分。"""
        if not all(math.isfinite(value) for value in (forward, requested, measured, dt)) or dt <= 0:
            self.reset()
            raise ValueError('角速度跟踪收到无效测量或时间间隔')
        dt = min(float(dt), .1)
        if self.filtered_rate is None:
            self.filtered_rate = float(measured)
        else:
            alpha = dt/(self.filter_tau+dt)
            self.filtered_rate += alpha*(measured-self.filtered_rate)
        error = requested-self.filtered_rate
        feedforward = self.feedforward_gain(forward)*requested
        tentative = max(-2., min(2., self.integral+error*dt))
        unbounded = feedforward+self.kp*error+self.ki*tentative
        # 只在误差把输出进一步推向饱和方向时冻结，避免积累不可执行的转向要求。
        if not ((unbounded > 1. and error > 0.) or (unbounded < -1. and error < 0.)):
            self.integral = tentative
        unbounded = feedforward+self.kp*error+self.ki*self.integral
        output = max(-1., min(1., unbounded))
        self.last = dict(requested_rate=float(requested), measured_rate=float(measured),
                         filtered_rate=float(self.filtered_rate), feedforward=float(feedforward),
                         error=float(error), integral=float(self.integral), output=float(output),
                         saturated=bool(abs(unbounded) >= 1.))
        return output

    def configuration(self):
        """公开实验参数和来源，报告不能只保存模式名称或源码哈希。"""
        return dict(kp=self.kp, ki=self.ki, filter_tau_s=self.filter_tau, input_limit=1.,
                    integral_limit=2., feedforward_gain_at_zero=2.4, feedforward_gain_at_vx_point5=1.6,
                    evidence='execution-region-20261004.json及逐帧heading基准；中间前速插值和反馈参数待独立验收',
                    validated_requested_rates=None)
