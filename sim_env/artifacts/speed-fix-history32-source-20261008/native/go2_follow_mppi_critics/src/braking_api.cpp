#include "braking_geometry.hpp"

extern "C"
{
// Python和MPPI共用同一套几何积分；接口不接收或读取仿真场景真值。
void * go2_geometry_create(const uint8_t * free, int width, int height, double resolution,
  double ox, double oy, double length, double body_width)
{
  try {return new go2_follow::Geometry(free, width, height, resolution, ox, oy, length, body_width);}
  catch (...) {return nullptr;}
}
void go2_geometry_destroy(void * handle) {delete static_cast<go2_follow::Geometry *>(handle);}
// 独立诊断可达停车域，不写回地图或替代最终停车保护。
int go2_geometry_seedable(void * handle, double x, double y, double yaw)
{
  return handle && static_cast<go2_follow::Geometry *>(handle)->seedable({x, y, yaw});
}
// 正距离出口只是软评分的充分条件；原搜索接入接口保留原布尔语义。
int go2_geometry_continuable(void * handle, double x, double y, double yaw, double distance)
{
  if (!handle || !std::isfinite(distance) || distance < 0. || distance > 2.) {return 0;}
  return static_cast<go2_follow::Geometry *>(handle)->seedable({x, y, yaw}, distance);
}
// 逐段执行原路线的直行/转身语义；折线不是新的自由证据。
int go2_geometry_route(void * handle, const double * xy, int count, double yaw,
  int has_final_yaw, double final_yaw)
{
  if (!handle || count <= 0) {return 0;}
  auto geometry = static_cast<go2_follow::Geometry *>(handle);
  std::vector<go2_follow::Point> route;
  for (int i = 0; i < count; ++i) {
    go2_follow::Point p{xy[2 * i], xy[2 * i + 1]};
    if (!route.empty() && std::hypot(p.x - route.back().x, p.y - route.back().y) < 1e-9) {continue;}
    if (route.size() > 1) {
      const auto before = route[route.size() - 2], at = route.back();
      const double ax = at.x - before.x, ay = at.y - before.y, bx = p.x - at.x, by = p.y - at.y;
      if (std::abs(ax * by - ay * bx) <= 1e-9 * std::hypot(ax, ay) * std::hypot(bx, by) &&
        ax * bx + ay * by > 0) {route.back() = p; continue;}
    }
    route.push_back(p);
  }
  go2_follow::Pose previous{route[0].x, route[0].y, yaw};
  if (!geometry->pose(previous)) {return 0;}
  for (size_t i = 1; i < route.size(); ++i) {
    const double heading = std::atan2(route[i].y - previous.y, route[i].x - previous.x);
    const go2_follow::Pose rotated{previous.x, previous.y, heading};
    const go2_follow::Pose next{route[i].x, route[i].y, heading};
    if (!geometry->motion(previous, rotated) || !geometry->motion(rotated, next)) {return 0;}
    previous = next;
  }
  return !has_final_yaw || geometry->motion(previous, {previous.x, previous.y, final_yaw});
}
// trace返回全部姿态及拒收端点，供现有诊断和离线回放使用。
int go2_geometry_brake(void * handle, double x, double y, double yaw, double v, double w,
  double lateral, double hold, double deceleration, double * trace, int capacity, double * info)
{
  if (!handle) {return -1;}
  const auto result = static_cast<go2_follow::Geometry *>(handle)->brake(
    {x, y, yaw}, v, w, lateral, hold, deceleration, 1., true);
  if (static_cast<int>(result.trace.size()) > capacity) {return -1;}
  info[0] = result.duration; info[1] = result.rejected_step;
  info[2] = static_cast<double>(result.trace.size());
  for (size_t i = 0; i < result.trace.size(); ++i) {
    trace[3 * i] = result.trace[i].x; trace[3 * i + 1] = result.trace[i].y;
    trace[3 * i + 2] = result.trace[i].yaw;
  }
  return result.safe ? 1 : 0;
}
}
