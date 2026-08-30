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

}  // namespace
}  // namespace scan_planner
