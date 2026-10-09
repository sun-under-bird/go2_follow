"""真实数据的单位、帧和数值检查，不刷新采集时间或放宽算法门槛。"""
import math
import numpy as np


def depth_in_meters(data, width, height, step, encoding, bigendian=False, integer_scale=.001):
    """保留行填充与端序；整数比例须来自驱动，零/异常深度保持无效。"""
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('depth must be 16UC1 or 32FC1')
    if not math.isfinite(integer_scale) or integer_scale <= 0:
        raise ValueError('integer depth scale must be finite and positive')
    itemsize = 2 if encoding == '16UC1' else 4
    if width <= 0 or height <= 0 or step < width * itemsize or step % itemsize or len(data) != height * step:
        raise ValueError('depth dimensions, row stride or payload length are invalid')
    dtype = ('>' if bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4')
    values = np.frombuffer(data, dtype=dtype).reshape(height, step // itemsize)[:, :width].astype(np.float32)
    if encoding == '16UC1':
        values *= integer_scale
    values[~np.isfinite(values) | (values <= 0)] = np.nan
    return np.ascontiguousarray(values, dtype='<f4')


def valid_stamp(stamp):
    return stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000 and (stamp.sec > 0 or stamp.nanosec > 0)


def validate_odometry(message, odom_frame, base_frame):
    """要求真实 pose/twist 的参考帧，拒绝非有限数值和无效四元数。"""
    if message.header.frame_id != odom_frame or message.child_frame_id != base_frame:
        raise ValueError('odometry frames differ from configured odom/base frames')
    if not valid_stamp(message.header.stamp):
        raise ValueError('odometry acquisition timestamp is invalid')
    p, q, v, w = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear, message.twist.twist.angular
    values = (p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('odometry contains nonfinite pose or velocity')
    if abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.) > .01:
        raise ValueError('odometry quaternion must have unit norm')


def transform_point(point, transform):
    """应用真实 TF 的旋转和平移，不能用改 frame_id 替代。"""
    q, t = transform.rotation, transform.translation
    values = (point.x, point.y, point.z, q.x, q.y, q.z, q.w, t.x, t.y, t.z)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('target or transform contains nonfinite values')
    if abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.) > .01:
        raise ValueError('transform quaternion must have unit norm')
    vector = np.array([point.x, point.y, point.z])
    axis = np.array([q.x, q.y, q.z])
    rotated = vector + 2 * np.cross(axis, np.cross(axis, vector) + q.w * vector)
    return rotated + np.array([t.x, t.y, t.z])


def cloud_xyz(message):
    """读取带端序、字段偏移和行填充的 XYZ 点云；坏帧不能充当空场景。"""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields and fields[name].count == 1 and fields[name].datatype in (7, 8) for name in ('x', 'y', 'z')):
        raise ValueError('obstacle cloud must contain scalar float XYZ fields')
    if message.width < 0 or message.height <= 0 or message.point_step <= 0 or message.row_step < message.width * message.point_step or len(message.data) != message.height * message.row_step:
        raise ValueError('obstacle cloud layout is invalid')
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('obstacle cloud field exceeds point stride')
        formats.append(('>' if message.is_bigendian else '<') + ('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)
    if message.width == 0:
        return np.empty((0, 3), dtype=float)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats, offsets=offsets, itemsize=message.point_step))
    view = np.ndarray((message.height, message.width), dtype=dtype, buffer=bytes(message.data),
                      strides=(message.row_step, message.point_step))
    points = np.column_stack([view[name].ravel() for name in ('x', 'y', 'z')]).astype(float)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('nonempty obstacle cloud contains no valid XYZ samples')
    return points


def transform_points(points, transform):
    """批量使用采集时刻 TF，返回 odom 中的点。"""
    q = transform.rotation
    # 复用严格 TF 检查，包括空点云的合法变换。
    origin = type('Point', (), dict(x=0., y=0., z=0.))()
    translation = transform_point(origin, transform)
    axis = np.array([q.x, q.y, q.z])
    return points + 2 * np.cross(axis, np.cross(axis, points) + q.w * points) + translation


def mark_obstacles(grid, points, stamp):
    """云只标记障碍，不生成自由空间、不刷新深度健康时间。"""
    if not len(points):
        return
    hit = points[(points[:, 2] >= .06) & (points[:, 2] <= .80)]
    indices = np.floor((hit[:, :2] - grid.origin) / grid.resolution).astype(int)
    indices = indices[np.all((indices >= 0) & (indices < grid.size), axis=1)]
    if len(indices):
        grid.occupied[indices[:, 1], indices[:, 0]] = True
        grid.seen[indices[:, 1], indices[:, 0]] = stamp
