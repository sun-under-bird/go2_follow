"""分别解释人方位、路径方向与机身朝向，避免把绕障方向差误报为测角误差。"""
import math
from .controller import wrap


def path_heading(route, position, lookahead=.6):
    """沿剩余路径取稳定前视点；零长度路线不虚构方向。"""
    if not route:
        return None
    remaining,anchor = lookahead,route[0]
    for point in route[1:]:
        length = math.dist(anchor,point)
        if length>1e-9:
            if remaining<=length:
                ratio = remaining/length
                destination = [anchor[i]+ratio*(point[i]-anchor[i]) for i in range(2)]
                break
            remaining -= length
        anchor = point
    else:
        destination = route[-1]
    dx,dy = destination[0]-position[0],destination[1]-position[1]
    return math.atan2(dy,dx) if math.hypot(dx,dy)>.03 else None


def heading_diagnostics(pose, target, route, velocity, command, motion, target_stamp=None, raw_target=None):
    """输出三种世界朝向、环绕角误差与角速度差，弧度读数附带采样时间。"""
    position = [pose.x,pose.y]
    bearing = None if target is None else math.atan2(target[1]-pose.y,target[0]-pose.x)
    path = path_heading(route,position)
    human_motion = math.atan2(velocity[1],velocity[0]) if math.hypot(*velocity)>.15 else None
    raw = None if raw_target is None else math.atan2(raw_target[1]-pose.y,raw_target[0]-pose.x)
    return dict(pose_t=pose.t,target_t=target_stamp,body_yaw=pose.yaw,target_bearing=bearing,
                raw_target_bearing=raw,path_heading=path,human_motion_heading=human_motion,
                target_error=None if bearing is None else wrap(bearing-pose.yaw),
                path_error=None if path is None else wrap(path-pose.yaw),
                target_path_difference=None if bearing is None or path is None else wrap(bearing-path),
                requested_wz=float(command[1]),measured_wz=float(motion[2]),
                rate_error=float(command[1]-motion[2]),
                reference='moving:path; stationary:person; single UWB tag does not measure human body heading')
