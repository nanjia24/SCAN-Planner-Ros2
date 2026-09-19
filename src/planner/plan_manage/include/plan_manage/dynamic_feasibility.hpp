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

template<typename FeasibilityCheck, typename Retime>
bool retimeUntilFeasible(
  const int max_retiming_attempts, FeasibilityCheck feasibility_check, Retime retime)
{
  for (int attempt = 0; attempt <= max_retiming_attempts; ++attempt) {
    double required_scale = 1.0;
    if (feasibility_check(&required_scale, attempt == max_retiming_attempts)) {
      return true;
    }
    if (attempt == max_retiming_attempts) {
      return false;
    }

    const double applied_scale = std::max(1.02, required_scale * 1.02);
    retime(applied_scale, attempt + 1, max_retiming_attempts);
  }
  return false;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__DYNAMIC_FEASIBILITY_HPP_
