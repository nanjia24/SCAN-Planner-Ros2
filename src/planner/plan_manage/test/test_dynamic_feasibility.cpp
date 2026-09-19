#include <gtest/gtest.h>

#include <plan_manage/dynamic_feasibility.hpp>

namespace scan_planner
{
namespace
{

TEST(DynamicFeasibility, UsesVelocityRatioWhenVelocityDominates)
{
  EXPECT_NEAR(requiredTimeScale(1.4, 1.0, 0.7, 1.0), 2.0, 1e-9);
}

TEST(DynamicFeasibility, UsesSquareRootRatioWhenAccelerationDominates)
{
  EXPECT_NEAR(requiredTimeScale(0.5, 4.0, 0.7, 1.0), 2.0, 1e-9);
}

TEST(DynamicFeasibility, DoesNotShortenAnAlreadyFeasibleTrajectory)
{
  EXPECT_DOUBLE_EQ(requiredTimeScale(0.5, 0.8, 0.7, 1.0), 1.0);
}

TEST(DynamicFeasibility, RechecksAfterRetimingAnInfeasibleTrajectory)
{
  int check_count = 0;
  int retime_count = 0;

  const bool feasible = retimeUntilFeasible(
    3,
    [&check_count](double *, bool) {
      ++check_count;
      return check_count >= 2;
    },
    [&retime_count](double, int, int) {
      ++retime_count;
    });

  EXPECT_TRUE(feasible);
  EXPECT_EQ(check_count, 2);
  EXPECT_EQ(retime_count, 1);
}

}  // namespace
}  // namespace scan_planner
