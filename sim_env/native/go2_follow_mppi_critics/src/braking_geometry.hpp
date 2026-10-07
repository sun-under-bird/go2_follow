#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace go2_follow
{
struct Point {double x, y;};
struct Pose {double x, y, yaw;};
struct BrakeResult {bool safe; double duration; int rejected_step; std::vector<Pose> trace; Pose stop;};

// 保存同一份相机自由证据；障碍、未知及窗口外均不可通行。
class Geometry
{
public:
  Geometry(const uint8_t * free, int width, int height, double resolution,
    double ox, double oy, double length, double body_width)
  : width_(width), height_(height), resolution_(resolution), ox_(ox), oy_(oy),
    length_(length), body_width_(body_width), free_(free, free + width * height),
    prefix_((width + 1) * (height + 1), 0)
  {
    for (int row = 0; row < height_; ++row) {
      int sum = 0;
      for (int col = 0; col < width_; ++col) {
        sum += !free_[row * width_ + col];
        prefix_[(row + 1) * (width_ + 1) + col + 1] =
          prefix_[row * (width_ + 1) + col + 1] + sum;
      }
    }
  }

  // 全自由外接框只提供快速充分证明；失败继续用完整矩形分离轴检查。
  bool window(int lx, int ly, int hx, int hy) const
  {
    if (lx < 0 || ly < 0 || hx >= width_ || hy >= height_) {return false;}
    const int stride = width_ + 1;
    return prefix_[(hy + 1) * stride + hx + 1] - prefix_[ly * stride + hx + 1] -
           prefix_[(hy + 1) * stride + lx] + prefix_[ly * stride + lx] == 0;
  }

  // 与Python版本逐格相同的闭矩形接触判定，包含内部障碍及擦角。
  bool pose(const Pose & p, double margin = 0.0) const
  {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    const double hl = length_ / 2 + margin, hw = body_width_ / 2 + margin;
    const double ex = std::abs(c) * hl + std::abs(s) * hw;
    const double ey = std::abs(s) * hl + std::abs(c) * hw;
    const int lx = low(p.x - ex, ox_), hx = high(p.x + ex, ox_);
    const int ly = low(p.y - ey, oy_), hy = high(p.y + ey, oy_);
    if (lx < 0 || ly < 0 || hx >= width_ || hy >= height_) {return false;}
    if (window(lx, ly, hx, hy)) {return true;}
    const double support = resolution_ / 2 * (std::abs(c) + std::abs(s));
    for (int row = ly; row <= hy; ++row) {
      for (int col = lx; col <= hx; ++col) {
        if (free_[row * width_ + col]) {continue;}
        const double dx = ox_ + (col + .5) * resolution_ - p.x;
        const double dy = oy_ + (row + .5) * resolution_ - p.y;
        if (std::abs(c * dx + s * dy) <= hl + support + 1e-10 &&
          std::abs(-s * dx + c * dy) <= hw + support + 1e-10) {return false;}
      }
    }
    return true;
  }

  // 固定朝向平移用首尾矩形凸包；旋转沿用中点膨胀误差界，覆盖连续扫角。
  bool motion(const Pose & a, const Pose & b, double margin = 0.0) const
  {
    const double extent = (length_ + body_width_) / 2 + 2 * margin;
    if (window(low(std::min(a.x, b.x) - extent, ox_),
      low(std::min(a.y, b.y) - extent, oy_), high(std::max(a.x, b.x) + extent, ox_),
      high(std::max(a.y, b.y) + extent, oy_))) {return true;}
    const double delta = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    if (std::abs(delta) < 1e-10) {
      auto vertices = rectangle(a, margin);
      vertices.reserve(8);
      const auto tail = rectangle(b, margin);
      vertices.insert(vertices.end(), tail.begin(), tail.end());
      return polygon(hull(vertices));
    }
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const double corner = std::hypot(length_, body_width_) / 2 + std::sqrt(2.) * margin;
    const int count = std::max(1, static_cast<int>(std::ceil(
      (distance + corner * std::abs(delta)) / (resolution_ * .25))));
    const double bound = distance / (2 * count) +
      2 * corner * std::sin(std::abs(delta) / (4 * count)) + 1e-9;
    for (int i = 0; i < count; ++i) {
      const double phase = (i + .5) / count;
      if (!pose({a.x + (b.x - a.x) * phase, a.y + (b.y - a.y) * phase,
        a.yaw + delta * phase}, margin + bound)) {return false;}
    }
    return true;
  }

  // 与最终执行层相同的保持/制动积分；不把请求转速当成已经建立的实测转速。
  BrakeResult brake(Pose p, double v, double w, double lateral, double hold,
    double deceleration, double angular_deceleration = 1., bool save_trace = false) const
  {
    const double duration = hold + std::max({std::abs(v) / deceleration,
      std::abs(w) / angular_deceleration, std::abs(lateral) / deceleration});
    BrakeResult result{true, duration, -1, {}, p};
    if (save_trace) {result.trace.push_back(p);}
    for (int i = 0; i < std::max(1, static_cast<int>(std::ceil(duration / .04))); ++i) {
      if (i * .04 >= hold) {
        v = std::copysign(std::max(0., std::abs(v) - deceleration * .04), v);
        lateral = std::copysign(std::max(0., std::abs(lateral) - deceleration * .04), lateral);
        w = std::copysign(std::max(0., std::abs(w) - angular_deceleration * .04), w);
      }
      const Pose next{p.x + (v * std::cos(p.yaw) - lateral * std::sin(p.yaw)) * .04,
        p.y + (v * std::sin(p.yaw) + lateral * std::cos(p.yaw)) * .04, p.yaw + w * .04};
      if (save_trace) {result.trace.push_back(next);}
      if (!motion(p, next)) {result.safe = false; result.rejected_step = i; return result;}
      p = next;
      result.stop = p;
    }
    return result;
  }

  // 停车姿态还须能重新接入局部姿态图；防止驶入“矩形安全但搜索无法起步”的盲区口袋。
  bool seedable(const Pose & p, double continuation = 0.) const
  {
    const double extent = (length_ + body_width_) / 2 + resolution_ * 1.5 + continuation;
    if (window(low(p.x - extent, ox_), low(p.y - extent, oy_),
      high(p.x + extent, ox_), high(p.y + extent, oy_))) {return true;}
    for (const double distance : {0., .2, .4, .6, .8, 1.}) {
      const Point at{p.x + distance * std::cos(p.yaw), p.y + distance * std::sin(p.yaw)};
      const int col = std::floor((at.x - ox_) / resolution_), row = std::floor((at.y - oy_) / resolution_);
      std::vector<Point> ends;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (col + dc < 0 || row + dr < 0 || col + dc >= width_ || row + dr >= height_) {continue;}
          ends.push_back({ox_ + (col + dc + .5) * resolution_, oy_ + (row + dr + .5) * resolution_});
        }
      }
      std::sort(ends.begin(), ends.end(), [at](Point a, Point b) {
        return std::hypot(a.x - at.x, a.y - at.y) < std::hypot(b.x - at.x, b.y - at.y);});
      if (distance == 0.) {ends = {{ox_ + (col + .5) * resolution_, oy_ + (row + .5) * resolution_}};}
      else if (ends.size() > 4) {ends.resize(4);}
      for (auto end : ends) {
        // 优先检查接近当前朝向的连接；仍遍历全部八个朝向，结果不因次序变化而放宽。
        // 停车评分每拍有800个候选，不能总先尝试远离当前朝向的完整转身。
        const int nearest = static_cast<int>(std::round(p.yaw / (std::acos(-1.) / 4)));
        for (const int offset : {0, 1, -1, 2, -2, 3, -3, 4}) {
          const int heading = ((nearest + offset) % 8 + 8) % 8;
          const double angle = heading * std::acos(-1.) / 4;
          if (!pose({end.x, end.y, angle})) {continue;}
          if (continuation > 0.) {
            // 仅接入一两个姿态仍可能没有续行出口。为评分提供已知直行出口的充分证明，
            // 不改矩形尺寸或地图；无此证明只增加软代价，不声明所有曲线路线都不可行。
            const Pose next{end.x + continuation * std::cos(angle),
              end.y + continuation * std::sin(angle), angle};
            if (!motion({end.x, end.y, angle}, next)) {continue;}
          }
          if (connect(p, {end}, angle) ||
            (distance > 0 && connect(p, {at, end}, angle))) {return true;}
        }
      }
    }
    return false;
  }

private:
  // 连续前缀必须完成各段朝向和矩形扫掠，再接入离散终点朝向。
  bool connect(Pose at, const std::vector<Point> & points, double final_yaw) const
  {
    for (auto point : points) {
      if (std::hypot(point.x - at.x, point.y - at.y) < 1e-9) {continue;}
      const double heading = std::atan2(point.y - at.y, point.x - at.x);
      const Pose rotated{at.x, at.y, heading}, next{point.x, point.y, heading};
      if (!motion(at, rotated) || !motion(rotated, next)) {return false;}
      at = next;
    }
    return motion(at, {at.x, at.y, final_yaw});
  }
  // 微小容差保留接触格的两侧，与现有Python矩形边界一致。
  int low(double x, double origin) const {return std::floor((x - origin) / resolution_ - 1e-9);}
  int high(double x, double origin) const {return std::floor((x - origin) / resolution_ + 1e-9);}
  // 连续位姿的四角仅用于生成凸包，碰撞仍检查全部接触格。
  std::vector<Point> rectangle(const Pose & p, double margin) const
  {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    std::vector<Point> points;
    points.reserve(4);
    for (const double x : {length_ / 2 + margin, -length_ / 2 - margin}) {
      for (const double y : {body_width_ / 2 + margin, -body_width_ / 2 - margin}) {
        points.push_back({p.x + c * x - s * y, p.y + s * x + c * y});
      }
    }
    return points;
  }
  // 计算有向面积，凸包删除共线内部点时使用。
  static double cross(Point a, Point b, Point c)
  {return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);}
  // 双精度单调链保留完整凸包，避免单精度舍入缩小擦边包络。
  static std::vector<Point> hull(std::vector<Point> points)
  {
    std::sort(points.begin(), points.end(), [](Point a, Point b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);});
    points.erase(std::unique(points.begin(), points.end(), [](Point a, Point b) {
      return a.x == b.x && a.y == b.y;}), points.end());
    std::vector<Point> lower, upper;
    for (auto p : points) {
      while (lower.size() > 1 && cross(lower[lower.size() - 2], lower.back(), p) <= 0.) {lower.pop_back();}
      lower.push_back(p);
    }
    for (auto i = points.rbegin(); i != points.rend(); ++i) {
      while (upper.size() > 1 && cross(upper[upper.size() - 2], upper.back(), *i) <= 0.) {upper.pop_back();}
      upper.push_back(*i);
    }
    lower.pop_back(); upper.pop_back(); lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
  }
  // 凸多边形与闭栅格方盒做完整SAT，不只查四个机身角点。
  bool polygon(const std::vector<Point> & vertices) const
  {
    double xmin = vertices[0].x, xmax = xmin, ymin = vertices[0].y, ymax = ymin;
    for (auto p : vertices) {
      xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    }
    const int lx = low(xmin, ox_), hx = high(xmax, ox_), ly = low(ymin, oy_), hy = high(ymax, oy_);
    if (lx < 0 || ly < 0 || hx >= width_ || hy >= height_) {return false;}
    if (window(lx, ly, hx, hy)) {return true;}
    for (int row = ly; row <= hy; ++row) {
      for (int col = lx; col <= hx; ++col) {
        if (free_[row * width_ + col]) {continue;}
        const Point center{ox_ + (col + .5) * resolution_, oy_ + (row + .5) * resolution_};
        if (center.x + resolution_ / 2 < xmin - 1e-10 ||
          center.x - resolution_ / 2 > xmax + 1e-10 ||
          center.y + resolution_ / 2 < ymin - 1e-10 ||
          center.y - resolution_ / 2 > ymax + 1e-10) {continue;}
        bool separated = false;
        for (size_t i = 0; i < vertices.size(); ++i) {
          const auto a = vertices[i], b = vertices[(i + 1) % vertices.size()];
          const double nx = -(b.y - a.y), ny = b.x - a.x;
          double minimum = nx * vertices[0].x + ny * vertices[0].y, maximum = minimum;
          for (auto p : vertices) {const double dot = nx * p.x + ny * p.y;
            minimum = std::min(minimum, dot); maximum = std::max(maximum, dot);}
          const double middle = nx * center.x + ny * center.y;
          const double support = resolution_ / 2 * (std::abs(nx) + std::abs(ny));
          if (middle + support < minimum - 1e-10 || middle - support > maximum + 1e-10) {
            separated = true; break;
          }
        }
        if (!separated) {return false;}
      }
    }
    return true;
  }
  int width_, height_;
  double resolution_, ox_, oy_, length_, body_width_;
  std::vector<uint8_t> free_;
  std::vector<int> prefix_;
};
}  // namespace go2_follow
