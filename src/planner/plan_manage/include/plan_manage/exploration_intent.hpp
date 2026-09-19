#ifndef PLAN_MANAGE_EXPLORATION_INTENT_HPP_
#define PLAN_MANAGE_EXPLORATION_INTENT_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace scan_planner
{

struct ExplorationTarget
{
  bool valid{false};
  bool fixed_home{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
};

struct ExplorationDirectionGate
{
  Eigen::Vector3d pending_direction{Eigen::Vector3d::Zero()};
  int pending_count{0};
};

struct ExplorationDirectionDecision
{
  bool accept{false};
  bool request_replan{false};
};

struct ExplorationReferencePrefix
{
  bool valid{false};
  double length{0.0};
  Eigen::Vector3d initial_direction{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_direction{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> waypoints;
};

inline ExplorationReferencePrefix buildOrderedExplorationPrefix(
    const Eigen::Vector3d &start,
    const std::vector<Eigen::Vector3d> &ordered_path,
    double planning_horizon)
{
  ExplorationReferencePrefix prefix;
  if (!start.allFinite() || ordered_path.size() < 2 ||
      !std::isfinite(planning_horizon) || planning_horizon <= 0.0)
  {
    return prefix;
  }
  if (!std::all_of(
          ordered_path.begin(), ordered_path.end(),
          [](const Eigen::Vector3d &point) { return point.allFinite(); }))
  {
    return prefix;
  }

  constexpr double duplicate_tolerance = 1e-3;
  constexpr double reanchor_distance = 0.5;
  std::size_t first_waypoint_index = 1;
  std::size_t closest_segment_index = 0;
  double closest_projection_ratio = 0.0;
  double closest_segment_distance_squared =
      std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < ordered_path.size(); ++index)
  {
    const Eigen::Vector2d segment =
        ordered_path[index + 1].head<2>() - ordered_path[index].head<2>();
    const double segment_length_squared = segment.squaredNorm();
    if (segment_length_squared <= duplicate_tolerance * duplicate_tolerance)
      continue;

    const double ratio = std::clamp(
        (start.head<2>() - ordered_path[index].head<2>()).dot(segment) /
            segment_length_squared,
        0.0, 1.0);
    const Eigen::Vector2d projection =
        ordered_path[index].head<2>() + ratio * segment;
    const double distance_squared =
        (start.head<2>() - projection).squaredNorm();
    const bool strictly_closer =
        distance_squared + 1e-9 < closest_segment_distance_squared;
    const bool passed_shared_adjacent_vertex =
        std::abs(distance_squared - closest_segment_distance_squared) <= 1e-9 &&
        index == closest_segment_index + 1 &&
        closest_projection_ratio >= 1.0 - 1e-6 && ratio <= 1e-6;
    if (strictly_closer || passed_shared_adjacent_vertex)
    {
      closest_segment_distance_squared = distance_squared;
      closest_segment_index = index;
      closest_projection_ratio = ratio;
      first_waypoint_index = index + 1;
    }
  }
  if (closest_segment_distance_squared > reanchor_distance * reanchor_distance)
    first_waypoint_index = 1;

  Eigen::Vector3d previous = start;
  prefix.waypoints.reserve(ordered_path.size() - first_waypoint_index);

  // TARE pose 0 is a delayed robot anchor. Re-anchor on the earliest closest
  // path segment so already passed points are removed without changing order.
  for (std::size_t index = first_waypoint_index;
       index < ordered_path.size(); ++index)
  {
    const Eigen::Vector3d delta = ordered_path[index] - previous;
    const double segment_length = delta.norm();
    if (segment_length <= duplicate_tolerance)
      continue;

    const double remaining = planning_horizon - prefix.length;
    if (remaining <= duplicate_tolerance)
      break;

    if (segment_length > remaining)
    {
      prefix.waypoints.push_back(previous + delta * (remaining / segment_length));
      prefix.length = planning_horizon;
      break;
    }

    prefix.waypoints.push_back(ordered_path[index]);
    prefix.length += segment_length;
    previous = ordered_path[index];
  }

  previous = start;
  for (const Eigen::Vector3d &waypoint : prefix.waypoints)
  {
    Eigen::Vector3d direction = waypoint - previous;
    direction.z() = 0.0;
    if (direction.head<2>().norm() > duplicate_tolerance)
    {
      direction.normalize();
      if (prefix.initial_direction.head<2>().norm() <= duplicate_tolerance)
        prefix.initial_direction = direction;
      prefix.terminal_direction = direction;
    }
    previous = waypoint;
  }

  prefix.valid = !prefix.waypoints.empty() &&
                 prefix.initial_direction.head<2>().norm() > duplicate_tolerance;
  return prefix;
}

inline double explorationPlanarDistance(
    const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs)
{
  return (lhs.head<2>() - rhs.head<2>()).norm();
}

inline ExplorationTarget computeExplorationTarget(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &waypoint,
    const Eigen::Vector3d &home,
    bool exploration_finished,
    double planning_horizon,
    const Eigen::Vector3d &map_origin,
    const Eigen::Vector3d &map_size,
    double map_margin,
    double home_radius)
{
  ExplorationTarget target;
  if (!start.allFinite() || !waypoint.allFinite() || !home.allFinite() ||
      !map_origin.allFinite() || !map_size.allFinite() ||
      planning_horizon <= 0.0 || map_margin < 0.0 || home_radius < 0.0)
  {
    return target;
  }

  const bool waypoint_is_home =
      explorationPlanarDistance(waypoint, home) <= home_radius;
  const bool vehicle_is_home =
      explorationPlanarDistance(start, home) <= home_radius;
  if (exploration_finished && (waypoint_is_home || vehicle_is_home))
  {
    target.valid = true;
    target.fixed_home = true;
    target.position = home;
    Eigen::Vector2d direction = home.head<2>() - start.head<2>();
    if (direction.norm() > 1e-6)
      target.direction.head<2>() = direction.normalized();
    return target;
  }

  Eigen::Vector2d direction = waypoint.head<2>() - start.head<2>();
  if (direction.norm() <= 1e-6)
    return target;
  direction.normalize();
  target.direction.head<2>() = direction;

  const Eigen::Vector2d lower = map_origin.head<2>().array() + map_margin;
  const Eigen::Vector2d upper =
      (map_origin + map_size).head<2>().array() - map_margin;
  if ((lower.array() >= upper.array()).any())
    return target;

  double distance = planning_horizon;
  for (int axis = 0; axis < 2; ++axis)
  {
    if (direction(axis) > 1e-9)
      distance = std::min(distance, (upper(axis) - start(axis)) / direction(axis));
    else if (direction(axis) < -1e-9)
      distance = std::min(distance, (lower(axis) - start(axis)) / direction(axis));
  }

  if (!std::isfinite(distance) || distance <= 1e-6)
    return target;

  target.valid = true;
  target.position = start;
  target.position.head<2>() += direction * distance;
  return target;
}

inline Eigen::Vector3d explorationTerminalVelocity(
    const ExplorationTarget &target, double max_velocity)
{
  if (!target.valid || target.fixed_home || max_velocity <= 0.0)
    return Eigen::Vector3d::Zero();

  Eigen::Vector3d direction = target.direction;
  direction.z() = 0.0;
  if (direction.head<2>().norm() <= 1e-6)
    return Eigen::Vector3d::Zero();
  return direction.normalized() * max_velocity;
}

inline bool explorationDirectionChanged(
    const Eigen::Vector3d &previous,
    const Eigen::Vector3d &candidate,
    double threshold_rad)
{
  const Eigen::Vector2d previous_xy = previous.head<2>();
  const Eigen::Vector2d candidate_xy = candidate.head<2>();
  if (previous_xy.norm() <= 1e-6 || candidate_xy.norm() <= 1e-6)
    return false;

  const double cosine = std::clamp(
      previous_xy.normalized().dot(candidate_xy.normalized()), -1.0, 1.0);
  return std::acos(cosine) > threshold_rad;
}

inline ExplorationDirectionDecision considerExplorationDirection(
    const Eigen::Vector3d &committed,
    const Eigen::Vector3d &candidate,
    double threshold_rad,
    int required_confirmations,
    ExplorationDirectionGate &gate)
{
  ExplorationDirectionDecision decision;
  const Eigen::Vector2d candidate_xy = candidate.head<2>();
  if (!candidate.allFinite() || candidate_xy.norm() <= 1e-6)
    return decision;

  const Eigen::Vector2d committed_xy = committed.head<2>();
  if (!committed.allFinite() || committed_xy.norm() <= 1e-6 ||
      !explorationDirectionChanged(committed, candidate, threshold_rad))
  {
    gate = ExplorationDirectionGate{};
    decision.accept = true;
    return decision;
  }

  if (required_confirmations <= 1)
  {
    gate = ExplorationDirectionGate{};
    decision.accept = true;
    decision.request_replan = true;
    return decision;
  }

  if (gate.pending_count <= 0 ||
      explorationDirectionChanged(
          gate.pending_direction, candidate, threshold_rad))
  {
    gate.pending_direction = candidate;
    gate.pending_direction.z() = 0.0;
    gate.pending_direction.normalize();
    gate.pending_count = 1;
    return decision;
  }

  gate.pending_direction = candidate;
  gate.pending_direction.z() = 0.0;
  gate.pending_direction.normalize();
  gate.pending_count++;
  if (gate.pending_count >= required_confirmations)
  {
    gate = ExplorationDirectionGate{};
    decision.accept = true;
    decision.request_replan = true;
  }
  return decision;
}

inline bool isExplorationWaypointFresh(
    double now_seconds, double waypoint_seconds, double timeout_seconds)
{
  const double age = now_seconds - waypoint_seconds;
  return timeout_seconds >= 0.0 && age >= 0.0 && age <= timeout_seconds;
}

inline bool shouldStartExplorationReplan(
    bool replan_requested, bool execution_frozen)
{
  return replan_requested && !execution_frozen;
}

inline bool allowExplorationWaypointFallback(
    bool fixed_home, bool ordered_reference_received)
{
  return fixed_home || !ordered_reference_received;
}

} // namespace scan_planner

#endif // PLAN_MANAGE_EXPLORATION_INTENT_HPP_
