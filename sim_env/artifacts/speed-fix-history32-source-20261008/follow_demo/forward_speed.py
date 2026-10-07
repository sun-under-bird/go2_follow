"""仅在MuJoCo中校准锁定步态的中高速线速度请求；不改Go2实机命令。"""
import math


class ForwardSpeedServo:
    """分开记录运动请求与策略内部输入，低速和全零请求保持原接口。"""
    def __init__(self, enabled=True):
        """前馈来自0.4/0.6/0.8开环实测，反馈只纠正剩余稳态误差。"""
        self.enabled = enabled
        self.reset()

    def reset(self):
        """停车和场景复位清积分，禁止下一次运动继承旧速度误差。"""
        self.integral, self.filtered = 0., None
        self.last = dict(requested=0., measured=0., model_input=0., active=False)

    @staticmethod
    def feedforward(requested):
        """反查锁定模型的中高速响应；末段允许有限外推，后续物理基准验证。"""
        actual = (.3402310265, .5306056923, .7098524291)
        inputs = (.4, .6, .8)
        index = 0 if requested <= actual[1] else 1
        return inputs[index] + (requested - actual[index]) * .2 / (actual[index + 1] - actual[index])

    def update(self, requested, measured, dt=.02):
        """中高速前馈加温和PI；0～0.3段不加入死区、最低速度或起步补偿。"""
        if not all(math.isfinite(v) for v in (requested, measured, dt)) or dt <= 0:
            self.reset()
            raise ValueError('仿真线速度标定收到无效输入')
        if not self.enabled or requested <= .3:
            self.reset()
            self.last = dict(requested=requested, measured=measured, model_input=requested, active=False)
            return requested
        dt = min(dt, .1)
        self.filtered = measured if self.filtered is None else self.filtered + dt / (.15 + dt) * (measured - self.filtered)
        # 0.3～0.4只平滑衔接标定区，不抬升低速请求。
        phase = min(1., (requested - .3) / .1)
        feedforward = requested + phase * (self.feedforward(requested) - requested)
        error = requested - self.filtered
        tentative = max(-.2, min(.2, self.integral + error * dt))
        unbounded = feedforward + phase * (.18 * error + .35 * tentative)
        if not ((unbounded > 1. and error > 0.) or (unbounded < 0. and error < 0.)):
            self.integral = tentative
        output = max(0., min(1., feedforward + phase * (.18 * error + .35 * self.integral)))
        self.last = dict(requested=requested, measured=measured, filtered=self.filtered, feedforward=feedforward,
                         error=error, integral=self.integral, model_input=output, active=True)
        return output
