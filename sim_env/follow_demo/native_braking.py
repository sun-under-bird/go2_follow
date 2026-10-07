"""复用MPPI插件内的矩形停车积分；可在无原生库的逻辑检查中使用原Python基准。"""
import ctypes
import math
from pathlib import Path
import numpy as np


def load_library():
    """只加载本实验台安装的库，缺库时不改变Python几何与停车边界。"""
    path = Path.home() / 'go2_sim/follow_native/lib/libgo2_follow_mppi_critics.so'
    if not path.exists():
        return None
    library = ctypes.CDLL(str(path))
    pointer, number, integer = ctypes.c_void_p, ctypes.c_double, ctypes.c_int
    library.go2_geometry_create.argtypes = [pointer, integer, integer] + [number] * 5
    library.go2_geometry_create.restype = pointer
    library.go2_geometry_destroy.argtypes = [pointer]
    library.go2_geometry_route.argtypes = [pointer, pointer, integer, number, integer, number]
    library.go2_geometry_route.restype = integer
    library.go2_geometry_seedable.argtypes = [pointer, number, number, number]
    library.go2_geometry_seedable.restype = integer
    library.go2_geometry_continuable.argtypes = [pointer, number, number, number, number]
    library.go2_geometry_continuable.restype = integer
    library.go2_geometry_brake.argtypes = [pointer] + [number] * 8 + [pointer, integer, pointer]
    library.go2_geometry_brake.restype = integer
    return library


class NativeBraking:
    """同一只读地图快照只建立一次索引；外部可写掩码每次重建，避免旧缓存漏障。"""
    def __init__(self):
        """原生库和状态归当前控制器拥有，地图不进入仿真或传感器线程。"""
        self.library = load_library()
        self.handle, self.key = None, None

    def close(self):
        """幂等释放私有几何副本，不创建进程、线程或ROS节点。"""
        if self.handle:
            self.library.go2_geometry_destroy(self.handle)
        self.handle, self.key = None, None

    def prepare(self, grid, free):
        """建立或复用当前完整自由证据索引；复制掩码后原生库不保留外部指针。"""
        metadata = (*grid.origin, grid.resolution, grid.footprint.length, grid.footprint.width, free.shape)
        if free.flags.writeable or self.key is None or self.key[0] is not free or self.key[1] != metadata:
            self.close()
            source = np.ascontiguousarray(free, dtype=np.uint8)
            self.handle = self.library.go2_geometry_create(source.ctypes.data, free.shape[1], free.shape[0],
                grid.resolution, *grid.origin, grid.footprint.length, grid.footprint.width)
            if not self.handle:
                raise RuntimeError('建立原生矩形地图失败')
            self.key = (free, metadata)

    def route(self, grid, free, route, yaw, final_yaw=None):
        """使用同一矩形扫掠校验接回路径；缺库时保留原Python判断。"""
        if self.library is None:
            return grid.route_clear(free,route,yaw,final_yaw)
        if not route:
            return False
        points = np.ascontiguousarray(route,dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all() or not math.isfinite(yaw):
            return False
        if final_yaw is not None and not math.isfinite(final_yaw):
            return False
        self.prepare(grid,free)
        return bool(self.library.go2_geometry_route(self.handle,points.ctypes.data,len(points),yaw,
                                                   final_yaw is not None,final_yaw or 0.))

    def brake(self, grid, free, pose, forward, turn, lateral, hold, deceleration):
        """返回与原保护相同的路线和拒收时刻；接口错误立即抛出，禁止默认为安全。"""
        if self.library is None:
            return None
        values = (pose.x, pose.y, pose.yaw, forward, turn, lateral, hold, deceleration)
        if not all(math.isfinite(value) for value in values) or hold < 0 or deceleration <= 0:
            raise ValueError('矩形停车积分收到无效输入')
        self.prepare(grid,free)
        duration = hold + max(abs(forward)/deceleration, abs(turn), abs(lateral)/deceleration)
        trace = np.empty((max(1, math.ceil(duration/.04))+1, 3), dtype=np.float64)
        info = np.empty(3, dtype=np.float64)
        safe = self.library.go2_geometry_brake(self.handle, *values, trace.ctypes.data, len(trace), info.ctypes.data)
        if safe < 0:
            raise RuntimeError('原生停车积分输出容量不一致')
        trace = trace[:int(info[2])]
        diagnostic = dict(forward=forward, turn=turn, lateral=lateral, hold_s=hold,
                          duration_s=float(info[0]), safe=bool(safe), backend='shared_native_geometry')
        if safe:
            diagnostic['stop_pose'] = trace[-1].tolist()
        if not safe:
            diagnostic.update(first_rejected_segment=trace[-2:, :2].tolist(),
                              elapsed_s=(int(info[1])+1)*.04)
            trace = trace[:-1]
        return bool(safe), trace[:, :2].tolist(), diagnostic

    def continuable(self, grid, free, pose, distance=.6):
        """核对停车后仍有已知直行出口；只使用原尺寸矩形及当前地图证据。"""
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,distance)) or not 0<=distance<=2.:
            raise ValueError('续行出口收到无效姿态或距离')
        if self.library is not None:
            self.prepare(grid,free)
            return bool(self.library.go2_geometry_continuable(self.handle,pose.x,pose.y,pose.yaw,distance))
        # 无原生库时使用相同连续接入前缀，不将缺库默认为可续行。
        for advance in (0.,.2,.4,.6,.8,1.):
            point=[pose.x+advance*math.cos(pose.yaw),pose.y+advance*math.sin(pose.yaw)]
            col,row=np.floor((np.asarray(point)-grid.origin)/grid.resolution).astype(int)
            cells=[(row+dr,col+dc) for dr in (-1,0,1) for dc in (-1,0,1)
                   if 0<=row+dr<grid.size and 0<=col+dc<grid.size]
            cells.sort(key=lambda cell:math.dist(grid.point(cell),point))
            cells=[(row,col)] if advance==0. else cells[:4]
            for cell in cells:
                end=grid.point(cell)
                for heading in range(8):
                    yaw=heading*math.pi/4
                    future=[end[0]+distance*math.cos(yaw),end[1]+distance*math.sin(yaw),yaw]
                    if not grid.motion_clear(free,[*end,yaw],future):
                        continue
                    if grid.route_clear(free,[[pose.x,pose.y],end],pose.yaw,yaw):
                        return True
                    if advance and grid.route_clear(free,[[pose.x,pose.y],point,end],pose.yaw,yaw):
                        return True
        return False
