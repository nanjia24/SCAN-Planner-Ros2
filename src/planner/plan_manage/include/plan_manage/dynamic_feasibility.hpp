#ifndef PLAN_MANAGE__DYNAMIC_FEASIBILITY_HPP_
#define PLAN_MANAGE__DYNAMIC_FEASIBILITY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner
{

inline double requiredTimeScale(
  const double max_velocity, const double max_acceleration,
  const double velocity_limit, const double acceleration_limit)
{
  if (velocity_limit <= 0.0 || acceleration_limit <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max({
    1.0,
    std::max(0.0, max_velocity) / velocity_limit,
    std::sqrt(std::max(0.0, max_acceleration) / acceleration_limit)});
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__DYNAMIC_FEASIBILITY_HPP_
