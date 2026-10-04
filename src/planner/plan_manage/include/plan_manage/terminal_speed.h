#pragma once
#include <algorithm>
#include <cmath>
namespace scan_planner {
// Reserve distance for command/vehicle lag before constant-deceleration braking.
// v * delay + v^2 / (2 * deceleration) <= distance - stop_distance.
inline double terminalSpeedLimit(double distance, double deceleration,
                                 double delay, double stop_distance) {
  const double remaining = std::max(0.0, distance - stop_distance);
  const double lag = deceleration * delay;
  // Rationalized form avoids cancellation close to the stopping point.
  const double denominator = std::sqrt(lag * lag + 2.0 * deceleration * remaining) + lag;
  return denominator > 0.0 ? 2.0 * deceleration * remaining / denominator : 0.0;
}
}  // namespace scan_planner
