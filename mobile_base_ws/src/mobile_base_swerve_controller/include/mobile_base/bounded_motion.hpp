#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace mobile_base
{
// 不依赖 ROS 的有界舵轮规划器，供实车控制器和回归测试共同使用。
struct BoundedMotion
{
  static constexpr double pi = 3.14159265358979323846;
  using Four = std::array<double, 4>;
  Four x{0.3, 0.3, -0.3, -0.3}, y{0.25, -0.25, 0.25, -0.25};
  // At the exact lateral boundary both +90° and -90° describe the same
  // rolling axis. A physical +5° probe confirmed that the front-left wheel
  // centre moves inward as the reported angle increases. Therefore the front
  // axle uses the opposite equivalent pair from the rear axle so all four
  // wheel centres move away from the wheelbase centre.
  std::array<int, 4> preferred_boundary_sign{-1, 1, 1, -1};
  Four angle{}, speed{};
  double radius{0.1}, limit{pi / 2}, alignment{5 * pi / 180};
  double steering_rate{0.15}, acceleration{8}, max_speed{20};
  double boundary_preference_hysteresis{5 * pi / 180};
  bool braking{false};
  double stopped_time{0};
  bool initialized{false};

  bool reset(const Four & actual)
  {
    initialized = false;
    speed.fill(0);
    for (double value : {radius, limit, alignment, steering_rate, acceleration, max_speed}) {
      if (!std::isfinite(value) || value <= 0) {return false;}
    }
    if (limit > pi / 2) {return false;}
    for (double q : actual) {
      if (!std::isfinite(q) || std::abs(q) > limit + 0.01) {return false;}
    }
    for (size_t i = 0; i < 4; ++i) {angle[i] = std::clamp(actual[i], -limit, limit);}
    speed.fill(0);
    braking = false;
    stopped_time = 0;
    initialized = true;
    return true;
  }

  // 只枚举限位内的等价目标；禁止用角度取模掩盖反馈累计圈数。
  static bool target(double vx, double vy, double actual, double held,
    double bound, double & q, double & velocity, int preferred_sign = 0,
    double preference_hysteresis = 0.0)
  {
    if (std::hypot(vx, vy) < 1e-4) {q = held; velocity = 0; return true;}
    double best = std::numeric_limits<double>::infinity();
    double preferred_cost = std::numeric_limits<double>::infinity();
    double preferred_q = 0.0, preferred_velocity = 0.0;
    const double raw = std::atan2(vy, vx);
    for (int k : {0, -1, 1, -2, 2}) {
      const double candidate = raw + k * pi;
      if (candidate < -bound - 1e-12 || candidate > bound + 1e-12) {continue;}
      const double cost = std::abs(candidate - actual);
      if (cost < best - 1e-12) {
        best = cost;
        q = std::clamp(candidate, -bound, bound);
        velocity = (std::abs(k) % 2 ? -1 : 1) * std::hypot(vx, vy);
      }
      if (preferred_sign != 0 && candidate * preferred_sign > 0.0 &&
        cost < preferred_cost)
      {
        preferred_cost = cost;
        preferred_q = std::clamp(candidate, -bound, bound);
        preferred_velocity = (std::abs(k) % 2 ? -1 : 1) * std::hypot(vx, vy);
      }
    }
    if (std::isfinite(preferred_cost) &&
      preferred_cost <= best + std::max(0.0, preference_hysteresis))
    {
      q = preferred_q;
      velocity = preferred_velocity;
    }
    return std::isfinite(best);
  }

  bool step(double vx, double vy, double wz, const Four & actual,
    const Four & measured_speed, double dt)
  {
    if (!initialized || !std::isfinite(vx) || !std::isfinite(vy) ||
      !std::isfinite(wz) || !std::isfinite(dt) || dt <= 0) {
      speed.fill(0); return false;
    }
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(actual[i]) || std::abs(actual[i]) > limit + 0.01 ||
        !std::isfinite(measured_speed[i])) {
        speed.fill(0); return false;
      }
    }
    dt = std::min(dt, 0.05);  // 调度停顿不能导致一次跨过很大的角度。
    const bool idle = std::hypot(vx, vy) < 1e-4 && std::abs(wz) < 1e-4;
    if (idle) {
      // 停车或超时取消尚未完成的转向，保持此刻实际角。
      reset(actual);
      return true;
    }
    Four goal{}, desired{};
    bool aligned = true, rolling = false;
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(x[i]) || !std::isfinite(y[i]) ||
        !std::isfinite(std::hypot(vx - wz * y[i], vy + wz * x[i]))) {
        speed.fill(0); return false;
      }
      if (!target(vx - wz * y[i], vy + wz * x[i], actual[i], angle[i],
        limit, goal[i], desired[i], preferred_boundary_sign[i],
        boundary_preference_hysteresis)) {
        reset(actual);  // 限位小于 90° 时可能无合法解：不夹角硬走。
        return true;
      }
      aligned = aligned && std::abs(goal[i] - actual[i]) <= alignment;
      rolling = rolling || std::abs(measured_speed[i]) > 0.05;
      desired[i] /= radius;
    }
    if (!aligned && rolling && !braking) {
      for (size_t i = 0; i < 4; ++i) {angle[i] = std::clamp(actual[i], -limit, limit);}
      braking = true;
      stopped_time = 0;
    }
    if (braking) {
      speed.fill(0);  // 驱动器执行减速；等待反馈，不能只看发出的零速命令。
      stopped_time = rolling ? 0 : stopped_time + dt;
      if (stopped_time < 0.15) {return true;}
      braking = false;
    }
    for (size_t i = 0; i < 4; ++i) {
      angle[i] = std::clamp(angle[i] +
        std::clamp(goal[i] - angle[i], -steering_rate * dt, steering_rate * dt),
        -limit, limit);
    }
    if (!aligned) {speed.fill(0); return true;}
    double largest = 0;
    for (size_t i = 0; i < 4; ++i) {
      desired[i] *= std::max(0.0, std::cos(goal[i] - actual[i]));
      largest = std::max(largest, std::abs(desired[i]));
    }
    const double scale = largest > max_speed ? max_speed / largest : 1;
    // 四轮共同插值，保持速度比例，避免各轮独立限加速度改变底盘运动方向。
    double delta = 0;
    for (size_t i = 0; i < 4; ++i) {
      desired[i] *= scale;
      delta = std::max(delta, std::abs(desired[i] - speed[i]));
    }
    const double fraction = delta > 0 ? std::min(1.0, acceleration * dt / delta) : 1;
    for (size_t i = 0; i < 4; ++i) {speed[i] += fraction * (desired[i] - speed[i]);}
    return true;
  }
};
}  // namespace mobile_base
