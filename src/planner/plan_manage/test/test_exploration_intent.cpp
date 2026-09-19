#include <gtest/gtest.h>

#include <plan_manage/exploration_intent.hpp>

namespace
{
using scan_planner::ExplorationTarget;
using scan_planner::ExplorationDirectionGate;
using scan_planner::allowExplorationWaypointFallback;
using scan_planner::buildOrderedExplorationPrefix;
using scan_planner::computeExplorationTarget;
using scan_planner::considerExplorationDirection;
using scan_planner::explorationDirectionChanged;
using scan_planner::explorationTerminalVelocity;
using scan_planner::isExplorationWaypointFresh;
using scan_planner::shouldStartExplorationReplan;

constexpr double kTolerance = 1e-6;

TEST(ExplorationIntent, ProjectsRollingTargetAlongPlanarDirection)
{
  const ExplorationTarget target = computeExplorationTarget(
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(6.0, 8.0, 4.0),
      Eigen::Vector3d::Zero(),
      false,
      5.0,
      Eigen::Vector3d(-10.0, -10.0, -2.0),
      Eigen::Vector3d(20.0, 20.0, 6.0),
      0.1,
      0.5);

  ASSERT_TRUE(target.valid);
  EXPECT_FALSE(target.fixed_home);
  EXPECT_NEAR(target.position.x(), 3.0, kTolerance);
  EXPECT_NEAR(target.position.y(), 4.0, kTolerance);
  EXPECT_NEAR(target.position.z(), 0.6, kTolerance);
  EXPECT_NEAR(target.direction.x(), 0.6, kTolerance);
  EXPECT_NEAR(target.direction.y(), 0.8, kTolerance);
  EXPECT_NEAR(target.direction.z(), 0.0, kTolerance);
}

TEST(ExplorationIntent, ClampsRollingTargetInsideMapMargin)
{
  const ExplorationTarget target = computeExplorationTarget(
      Eigen::Vector3d(4.0, 0.0, 0.4),
      Eigen::Vector3d(20.0, 0.0, 0.4),
      Eigen::Vector3d::Zero(),
      false,
      7.5,
      Eigen::Vector3d(-5.0, -5.0, -1.0),
      Eigen::Vector3d(10.0, 10.0, 3.0),
      0.2,
      0.5);

  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.position.x(), 4.8, kTolerance);
  EXPECT_NEAR(target.position.y(), 0.0, kTolerance);
  EXPECT_NEAR(target.position.z(), 0.4, kTolerance);
}

TEST(ExplorationIntent, KeepsRollingDuringReturnUntilHomeIsSelected)
{
  const ExplorationTarget target = computeExplorationTarget(
      Eigen::Vector3d(8.0, 0.0, 0.4),
      Eigen::Vector3d(3.0, 0.0, 0.4),
      Eigen::Vector3d(0.0, 0.0, 0.4),
      true,
      4.0,
      Eigen::Vector3d(-10.0, -10.0, -1.0),
      Eigen::Vector3d(20.0, 20.0, 3.0),
      0.1,
      0.5);

  ASSERT_TRUE(target.valid);
  EXPECT_FALSE(target.fixed_home);
  EXPECT_NEAR(target.position.x(), 4.0, kTolerance);
}

TEST(ExplorationIntent, SelectsFixedHomeWhenReturnWaypointMatchesHome)
{
  const Eigen::Vector3d home(1.0, 2.0, 0.4);
  const ExplorationTarget target = computeExplorationTarget(
      Eigen::Vector3d(8.0, 2.0, 0.4),
      Eigen::Vector3d(1.3, 2.0, 0.4),
      home,
      true,
      4.0,
      Eigen::Vector3d(-10.0, -10.0, -1.0),
      Eigen::Vector3d(20.0, 20.0, 3.0),
      0.1,
      0.5);

  ASSERT_TRUE(target.valid);
  EXPECT_TRUE(target.fixed_home);
  EXPECT_TRUE(target.position.isApprox(home, kTolerance));
}

TEST(ExplorationIntent, SelectsFixedHomeWhenVehicleHasReachedHomeRadius)
{
  const Eigen::Vector3d home(1.0, 2.0, 0.4);
  const ExplorationTarget target = computeExplorationTarget(
      Eigen::Vector3d(1.2, 2.0, 0.4),
      Eigen::Vector3d(8.0, 2.0, 0.4),
      home,
      true,
      4.0,
      Eigen::Vector3d(-10.0, -10.0, -1.0),
      Eigen::Vector3d(20.0, 20.0, 3.0),
      0.1,
      0.5);

  ASSERT_TRUE(target.valid);
  EXPECT_TRUE(target.fixed_home);
  EXPECT_TRUE(target.position.isApprox(home, kTolerance));
}

TEST(ExplorationIntent, UsesNonzeroVelocityOnlyForRollingTarget)
{
  ExplorationTarget rolling;
  rolling.valid = true;
  rolling.direction = Eigen::Vector3d(0.6, 0.8, 0.0);

  const Eigen::Vector3d rolling_velocity = explorationTerminalVelocity(rolling, 0.7);
  EXPECT_NEAR(rolling_velocity.x(), 0.42, kTolerance);
  EXPECT_NEAR(rolling_velocity.y(), 0.56, kTolerance);

  rolling.fixed_home = true;
  EXPECT_TRUE(explorationTerminalVelocity(rolling, 0.7).isZero(kTolerance));
}

TEST(ExplorationIntent, DetectsOnlyLargePlanarDirectionChanges)
{
  const Eigen::Vector3d forward(1.0, 0.0, 0.0);
  const Eigen::Vector3d small_change(std::cos(0.2), std::sin(0.2), 3.0);
  const Eigen::Vector3d large_change(std::cos(0.8), std::sin(0.8), -2.0);

  EXPECT_FALSE(explorationDirectionChanged(forward, small_change, 0.5));
  EXPECT_TRUE(explorationDirectionChanged(forward, large_change, 0.5));
}

TEST(ExplorationIntent, HoldsSingleLargeDirectionChange)
{
  ExplorationDirectionGate gate;
  const auto decision = considerExplorationDirection(
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      0.5,
      2,
      gate);

  EXPECT_FALSE(decision.accept);
  EXPECT_FALSE(decision.request_replan);
  EXPECT_EQ(gate.pending_count, 1);
}

TEST(ExplorationIntent, AcceptsConfirmedLargeDirectionChange)
{
  ExplorationDirectionGate gate;
  const Eigen::Vector3d committed(1.0, 0.0, 0.0);
  EXPECT_FALSE(considerExplorationDirection(
                   committed, Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5, 2, gate)
                   .accept);

  const auto decision = considerExplorationDirection(
      committed, Eigen::Vector3d(-1.0, -0.1, 0.0), 0.5, 2, gate);

  EXPECT_TRUE(decision.accept);
  EXPECT_TRUE(decision.request_replan);
  EXPECT_EQ(gate.pending_count, 0);
}

TEST(ExplorationIntent, AcceptsSameDirectionImmediately)
{
  ExplorationDirectionGate gate;
  EXPECT_FALSE(considerExplorationDirection(
                   Eigen::Vector3d(1.0, 0.0, 0.0),
                   Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5, 2, gate)
                   .accept);

  const auto decision = considerExplorationDirection(
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.1, 0.0),
      0.5,
      2,
      gate);

  EXPECT_TRUE(decision.accept);
  EXPECT_FALSE(decision.request_replan);
  EXPECT_EQ(gate.pending_count, 0);
}

TEST(ExplorationIntent, RejectsExpiredAndFutureWaypoints)
{
  EXPECT_TRUE(isExplorationWaypointFresh(10.0, 8.0, 3.0));
  EXPECT_TRUE(isExplorationWaypointFresh(10.0, 7.0, 3.0));
  EXPECT_FALSE(isExplorationWaypointFresh(10.0, 6.9, 3.0));
  EXPECT_FALSE(isExplorationWaypointFresh(10.0, 10.1, 3.0));
}

TEST(ExplorationIntent, DefersRequestedReplanWhileExecutionIsFrozen)
{
  EXPECT_FALSE(shouldStartExplorationReplan(true, true));
  EXPECT_TRUE(shouldStartExplorationReplan(true, false));
  EXPECT_FALSE(shouldStartExplorationReplan(false, false));
}

TEST(ExplorationIntent, DoesNotFallBackToWaypointAfterOrderedReferenceStarts)
{
  EXPECT_FALSE(allowExplorationWaypointFallback(false, true));
  EXPECT_TRUE(allowExplorationWaypointFallback(false, false));
  EXPECT_TRUE(allowExplorationWaypointFallback(true, true));
}

TEST(ExplorationIntent, PreservesSourceOrderAtSelfIntersection)
{
  const std::vector<Eigen::Vector3d> ordered_path{
      {0.0, 0.0, 0.4},
      {2.0, 0.0, 0.4},
      {2.0, 2.0, 0.4},
      {0.0, 2.0, 0.4},
      {0.0, 0.0, 0.4},
      {-2.0, 0.0, 0.4},
  };

  const auto prefix = buildOrderedExplorationPrefix(
      Eigen::Vector3d(0.1, 0.0, 0.4), ordered_path, 10.0);

  ASSERT_TRUE(prefix.valid);
  ASSERT_EQ(prefix.waypoints.size(), 5U);
  EXPECT_TRUE(prefix.waypoints[0].isApprox(
      Eigen::Vector3d(2.0, 0.0, 0.4), kTolerance));
  EXPECT_TRUE(prefix.waypoints[3].isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.4), kTolerance));
  EXPECT_TRUE(prefix.waypoints[4].isApprox(
      Eigen::Vector3d(-2.0, 0.0, 0.4), kTolerance));
}

TEST(ExplorationIntent, ClipsOrderedPathAtPlanningHorizon)
{
  const std::vector<Eigen::Vector3d> ordered_path{
      {0.0, 0.0, 0.4},
      {2.0, 0.0, 0.4},
      {2.0, 3.0, 0.4},
  };

  const auto prefix = buildOrderedExplorationPrefix(
      Eigen::Vector3d(0.0, 0.0, 0.4), ordered_path, 3.0);

  ASSERT_TRUE(prefix.valid);
  ASSERT_EQ(prefix.waypoints.size(), 2U);
  EXPECT_TRUE(prefix.waypoints[0].isApprox(
      Eigen::Vector3d(2.0, 0.0, 0.4), kTolerance));
  EXPECT_TRUE(prefix.waypoints[1].isApprox(
      Eigen::Vector3d(2.0, 1.0, 0.4), kTolerance));
  EXPECT_NEAR(prefix.length, 3.0, kTolerance);
}

TEST(ExplorationIntent, SkipsOrderedWaypointsAlreadyPassedByCurrentOdometry)
{
  const std::vector<Eigen::Vector3d> ordered_path{
      {16.44, -8.14, 0.47},
      {15.75, -8.27, 0.42},
      {14.75, -8.27, 0.34},
      {13.75, -8.27, 0.36},
  };

  const auto prefix = buildOrderedExplorationPrefix(
      Eigen::Vector3d(15.09, -8.28, 0.40), ordered_path, 5.0);

  ASSERT_TRUE(prefix.valid);
  ASSERT_EQ(prefix.waypoints.size(), 2U);
  EXPECT_TRUE(prefix.waypoints[0].isApprox(
      Eigen::Vector3d(14.75, -8.27, 0.34), kTolerance));
  EXPECT_LT(prefix.initial_direction.x(), -0.99);
}

TEST(ExplorationIntent, TreatsSharedAdjacentVertexAsAlreadyPassed)
{
  const std::vector<Eigen::Vector3d> ordered_path{
      {0.0, 0.0, 0.4},
      {1.0, 0.0, 0.4},
      {2.0, -1.0, 0.4},
  };

  const auto prefix = buildOrderedExplorationPrefix(
      Eigen::Vector3d(1.0, 0.15, 0.4), ordered_path, 5.0);

  ASSERT_TRUE(prefix.valid);
  ASSERT_EQ(prefix.waypoints.size(), 1U);
  EXPECT_TRUE(prefix.waypoints.front().isApprox(
      Eigen::Vector3d(2.0, -1.0, 0.4), kTolerance));
}

TEST(ExplorationIntent, UsesFirstAndLastOrderedSegmentsAsDirections)
{
  const std::vector<Eigen::Vector3d> ordered_path{
      {0.0, 0.0, 0.4},
      {2.0, 0.0, 0.4},
      {2.0, 2.0, 0.4},
  };

  const auto prefix = buildOrderedExplorationPrefix(
      Eigen::Vector3d(0.0, 0.0, 0.4), ordered_path, 5.0);

  ASSERT_TRUE(prefix.valid);
  EXPECT_TRUE(prefix.initial_direction.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), kTolerance));
  EXPECT_TRUE(prefix.terminal_direction.isApprox(
      Eigen::Vector3d(0.0, 1.0, 0.0), kTolerance));
}
} // namespace
