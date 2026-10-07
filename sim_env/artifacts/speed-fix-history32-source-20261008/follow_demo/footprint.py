"""矩形机身与栅格方块的完整相交检查，统一规划、转向和执行口径。"""
from dataclasses import dataclass
from functools import lru_cache
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class RectangleFootprint:
    """以 base_footprint 原点为中心的矩形；长轴沿机身 x，宽轴沿 y。"""
    length: float = .70
    width: float = .32

    def __post_init__(self):
        """拒绝非有限或非正尺寸，不能将坏配置降级为一个中心点。"""
        if not all(math.isfinite(v) and v > 0 for v in (self.length, self.width)):
            raise ValueError('矩形机身长宽必须是有限正数')

    @property
    def corner_distance(self):
        """返回最远角到中心的距离，仅用于旋转采样误差界，不用圆形判碰。"""
        return math.hypot(self.length, self.width)/2

    def vertices(self, x, y, yaw, margin=0.):
        """生成实际朝向的四个角；额外余量分别沿矩形长宽方向扩展。"""
        a, b = self.length/2+margin, self.width/2+margin
        local = np.array([[a,b],[-a,b],[-a,-b],[a,-b]])
        c, s = math.cos(yaw), math.sin(yaw)
        return local @ np.array([[c,s],[-s,c]]) + [x,y]


def polygon_cells(vertices, origin, resolution, size=None):
    """用分离轴定理找出凸多边形接触的全部栅格，包含内部、擦边与擦角。"""
    vertices = np.asarray(vertices, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or not np.isfinite(vertices).all():
        return None
    low = np.floor((vertices.min(axis=0)-origin)/resolution-1e-9).astype(int)
    high = np.floor((vertices.max(axis=0)-origin)/resolution+1e-9).astype(int)
    if size is not None and (np.any(low < 0) or np.any(high >= size)):
        return None
    cols, rows = np.meshgrid(np.arange(low[0],high[0]+1), np.arange(low[1],high[1]+1))
    centers = np.column_stack((cols.ravel(),rows.ravel()))*resolution+origin+resolution/2
    touched = np.ones(len(centers),dtype=bool)
    edges = np.roll(vertices,-1,axis=0)-vertices
    axes = np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
    # 多边形边法向及栅格 x/y 都要检查；只检查四个角会漏掉内部障碍。
    for axis in axes:
        if np.linalg.norm(axis) < 1e-12:
            continue
        projection = vertices @ axis
        center = centers @ axis
        extent = resolution/2*np.abs(axis).sum()
        touched &= (center+extent >= projection.min()-1e-10) & (center-extent <= projection.max()+1e-10)
    return np.column_stack((rows.ravel()[touched],cols.ravel()[touched]))


def polygon_clear(grid, free, vertices):
    """完整多边形必须落在已知自由格内，窗口外同样拒绝。"""
    vertices = np.asarray(vertices,dtype=float)
    low = np.floor((vertices.min(axis=0)-grid.origin)/grid.resolution-1e-9).astype(int)
    high = np.floor((vertices.max(axis=0)-grid.origin)/grid.resolution+1e-9).astype(int)
    if np.any(low<0) or np.any(high>=grid.size):
        return False
    area = free[low[1]:high[1]+1,low[0]:high[0]+1]
    if np.all(area):
        # 包围盒全部自由就已证明整个多边形自由，避免为大量空地分配完整触碰格数组。
        return True
    bad = np.argwhere(~area)
    centers = (bad[:,::-1]+low+.5)*grid.resolution+grid.origin
    edges = np.roll(vertices,-1,axis=0)-vertices
    axes = np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
    axes = axes[np.linalg.norm(axes,axis=1)>1e-12]
    projection,cell_projection = vertices@axes.T,centers@axes.T
    extent = grid.resolution/2*np.abs(axes).sum(axis=1)
    touched = np.all((cell_projection+extent >= projection.min(axis=0)-1e-10)
                     & (cell_projection-extent <= projection.max(axis=0)+1e-10),axis=1)
    return not bool(np.any(touched))


def convex_hull(vertices):
    """用双精度单调链求凸包，避免 float32 舍入将恰好擦边的矩形缩小。"""
    points = sorted(set(map(tuple,np.asarray(vertices,dtype=float))))
    def side(a,b,c):
        """计算二维有向面积，删除凸包边内部的重复共线点。"""
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower,upper = [],[]
    for point in points:
        while len(lower)>1 and side(lower[-2],lower[-1],point)<=0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper)>1 and side(upper[-2],upper[-1],point)<=0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1]+upper[:-1])


def pose_clear(grid, free, x, y, yaw, margin=0.):
    """按连续实测位置和朝向检查矩形，不能用所在格中心替代机器人位置。"""
    if not all(math.isfinite(v) for v in (x,y,yaw,margin)) or margin < 0:
        return False
    c,s = math.cos(yaw),math.sin(yaw)
    half_length,half_width = grid.footprint.length/2+margin,grid.footprint.width/2+margin
    extent_x,extent_y = abs(c)*half_length+abs(s)*half_width,abs(s)*half_length+abs(c)*half_width
    low_x = math.floor((x-extent_x-grid.origin[0])/grid.resolution-1e-9)
    high_x = math.floor((x+extent_x-grid.origin[0])/grid.resolution+1e-9)
    low_y = math.floor((y-extent_y-grid.origin[1])/grid.resolution-1e-9)
    high_y = math.floor((y+extent_y-grid.origin[1])/grid.resolution+1e-9)
    if min(low_x,low_y)<0 or max(high_x,high_y)>=grid.size:
        return False
    if grid.window_free(free,low_x,low_y,high_x,high_y):
        return True
    area = free[low_y:high_y+1,low_x:high_x+1]
    bad = np.argwhere(~area)
    dx = (bad[:,1]+low_x+.5)*grid.resolution+grid.origin[0]-x
    dy = (bad[:,0]+low_y+.5)*grid.resolution+grid.origin[1]-y
    support = grid.resolution/2*(abs(c)+abs(s))
    # 包围盒已检查世界x/y分离轴，再检查矩形自身长宽两轴，等价于完整SAT。
    intersects = ((abs(c*dx+s*dy)<=half_length+support+1e-10)
                  & (abs(-s*dx+c*dy)<=half_width+support+1e-10))
    return not bool(np.any(intersects))


def motion_clear(grid, free, a, b, margin=0.):
    """检查两个 SE(2) 位姿之间的完整扫掠，旋转区间用有误差界的膨胀覆盖。"""
    if (len(a) != 3 or len(b) != 3 or not np.isfinite([a,b]).all()
            or not math.isfinite(margin) or margin < 0.):
        return False
    ax,ay,heading = map(float,a)
    bx,by,end_heading = map(float,b)
    # 任意旋转矩形都位于此外接方框内；整个平移扫掠的外接框全自由即可直接证明安全。
    # 这不是新机身包络，也不拒绝窄通道：框中有禁行格时继续下面原有矩形SAT/扫角检查。
    extent = (grid.footprint.length+grid.footprint.width)/2+2*margin
    low_x = math.floor((min(ax,bx)-extent-grid.origin[0])/grid.resolution-1e-9)
    high_x = math.floor((max(ax,bx)+extent-grid.origin[0])/grid.resolution+1e-9)
    low_y = math.floor((min(ay,by)-extent-grid.origin[1])/grid.resolution-1e-9)
    high_y = math.floor((max(ay,by)+extent-grid.origin[1])/grid.resolution+1e-9)
    if grid.window_free(free,low_x,low_y,high_x,high_y):
        return True
    delta = math.atan2(math.sin(end_heading-heading),math.cos(end_heading-heading))
    if abs(delta) < 1e-10:
        # 固定朝向平移的扫掠就是首尾矩形的凸包，精确覆盖两次采样之间的薄障碍。
        vertices = np.vstack((grid.footprint.vertices(ax,ay,heading,margin),
                              grid.footprint.vertices(bx,by,heading,margin)))
        hull = convex_hull(vertices)
        return polygon_clear(grid,free,hull)
    distance = math.hypot(bx-ax,by-ay)
    corner = grid.footprint.corner_distance+math.sqrt(2)*margin
    count = max(1,math.ceil((distance+corner*abs(delta))/(grid.resolution*.25)))
    # 区间内任一点距中点的位移，至多为半段平移 + 角点旋转弦长。
    # 将此距离加到长宽半尺寸，覆盖整个连续区间及两个端点，不能仅采样角度而漏掉中途扫角。
    bound = distance/(2*count)+2*corner*math.sin(abs(delta)/(4*count))+1e-9
    for index in range(count):
        phase = (index+.5)/count
        if not pose_clear(grid,free,ax+(bx-ax)*phase,ay+(by-ay)*phase,
                          heading+delta*phase,margin+bound):
            return False
    return True


def route_clear(grid, free, route, initial_yaw, final_yaw=None, margin=0.):
    """逐段检查直行及连接处转身的矩形扫掠；MPPI 可在此可达路线附近平滑跟踪。"""
    if not route:
        return False
    compact = [route[0]]
    for point in route[1:]:
        if math.dist(compact[-1],point)<1e-9:
            continue
        if len(compact)>1:
            before = np.asarray(compact[-1])-compact[-2]
            after = np.asarray(point)-compact[-1]
            cross = before[0]*after[1]-before[1]*after[0]
            if abs(cross) <= 1e-9*np.linalg.norm(before)*np.linalg.norm(after) and before@after>0:
                compact[-1] = point
                continue
        compact.append(point)
    route = compact
    heading = initial_yaw
    if not pose_clear(grid,free,*route[0],heading,margin):
        return False
    for a,b in zip(route,route[1:]):
        if math.dist(a,b) < 1e-9:
            continue
        desired = math.atan2(b[1]-a[1],b[0]-a[0])
        if not motion_clear(grid,free,[*a,heading],[*a,desired],margin):
            return False
        if not motion_clear(grid,free,[*a,desired],[*b,desired],margin):
            return False
        heading = desired
    return final_yaw is None or motion_clear(grid,free,[*route[-1],heading],[*route[-1],final_yaw],margin)


@lru_cache(maxsize=64)
def lattice_kernels(length, width, resolution, headings=8, margin=0.):
    """预计算姿态、前进和相邻转向的矩形扫掠核，地图变化只需 OpenCV 腐蚀。"""
    body = RectangleFootprint(length,width)
    angle_step = 2*math.pi/headings
    extent = math.ceil((body.corner_distance+resolution*math.sqrt(2)+math.sqrt(2)*margin)/resolution)+2
    size = 2*extent+1
    origin = np.array([-extent-.5,-extent-.5])*resolution

    def raster(vertices):
        """将凸包完整接触格写入奇数核，核中心对应搜索栅格中心。"""
        kernel = np.zeros((size,size),np.uint8)
        cells = polygon_cells(vertices,origin,resolution,size)
        kernel[cells[:,0],cells[:,1]] = 1
        return kernel

    poses,moves,turns,steps = [],[],[],[]
    for index in range(headings):
        heading = index*angle_step
        dc,dr = round(math.cos(heading)),round(math.sin(heading))
        steps.append((dr,dc))
        poses.append(raster(body.vertices(0,0,heading,margin)))
        vertices = np.vstack((body.vertices(0,0,heading,margin),
                              body.vertices(dc*resolution,dr*resolution,heading,margin)))
        moves.append(raster(convex_hull(vertices)))
        count = max(1,math.ceil(body.corner_distance*angle_step/(resolution*.25)))
        bound = 2*(body.corner_distance+math.sqrt(2)*margin)*math.sin(angle_step/(4*count))+1e-9
        turn = poses[-1].copy()
        turn |= raster(body.vertices(0,0,heading+angle_step,margin))
        for sample in range(count):
            turn |= raster(body.vertices(0,0,heading+(sample+.5)*angle_step/count,margin+bound))
        turns.append(turn)
    return poses,moves,turns,steps


def lattice_layers(grid, free, margin=0.):
    """返回八个朝向的可站立、可前进、可转向图；二维并集只用于显示和评分。"""
    kernels = lattice_kernels(grid.footprint.length,grid.footprint.width,grid.resolution,8,margin)
    source = free.astype(np.uint8)
    groups = [np.array([cv2.erode(source,k,borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
                       for k in group]) for group in kernels[:3]]
    return (*groups,kernels[3])
