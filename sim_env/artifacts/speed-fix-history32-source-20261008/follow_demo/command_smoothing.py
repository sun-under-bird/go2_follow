"""跟踪命令的舒适性平滑；目标改变时不能继续沿旧加速度远离目标。"""
import math
from .controller import clamp


def smooth_axis(previous, acceleration, desired, dt, acceleration_limit, jerk_limit, lower, upper):
    """在请求范围内追踪一个速度轴，返回命令与下一拍加速度状态。

    正常追踪限制加速度和加速度变化。目标突然反向变化时，旧加速度
    可能将输出推得比上一拍更远；此时撤销该残余状态。这个投影优先于
    舒适性jerk限制，与硬限幅一样必须显式记录，而不是在停车保护处补救。
    """
    values=(previous,acceleration,desired,dt,acceleration_limit,jerk_limit,lower,upper)
    if not all(math.isfinite(v) for v in values) or min(dt,acceleration_limit,jerk_limit)<=0 or lower>upper:
        raise ValueError('平滑器输入须有限，时间及变化率为正，边界须有序')
    desired=clamp(desired,lower,upper)
    error=desired-previous
    reversed_acceleration=error*acceleration<0
    if reversed_acceleration:
        acceleration=0.
    request=clamp(error/dt,-acceleration_limit,acceleration_limit)
    acceleration+=clamp(request-acceleration,-jerk_limit*dt,jerk_limit*dt)
    candidate=previous+acceleration*dt
    # 防止越过目标；输出边界仍独立有效，不由误差或平滑状态替代。
    if error*(desired-candidate)<=0:
        candidate,acceleration=desired,0.
    bounded=clamp(candidate,lower,upper)
    if bounded!=candidate:
        acceleration=0.
    return bounded,acceleration,reversed_acceleration
