#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

namespace scan_planner {
// Feedback only: preserves the requested observation orientation, which the
// SCAN position planner does not execute. No terrain/admission policy here.
inline bool goalPositionAdjusted(const geometry_msgs::msg::Pose &requested,
                                 const geometry_msgs::msg::Pose &effective) {
  constexpr double tolerance = 1e-6;
  return std::abs(requested.position.x - effective.position.x) > tolerance ||
         std::abs(requested.position.y - effective.position.y) > tolerance ||
         std::abs(requested.position.z - effective.position.z) > tolerance;
}
}  // namespace scan_planner
