#include <gtest/gtest.h>
#include <plan_manage/goal_feedback.hpp>

TEST(GoalFeedback, DetectsHorizontalAndHeightPolicyChanges) {
  geometry_msgs::msg::Pose requested;
  requested.position.x = 2.0;
  requested.position.z = 0.15;
  requested.orientation.z = 0.5;
  requested.orientation.w = 0.866025403784;
  auto effective = requested;
  EXPECT_FALSE(scan_planner::goalPositionAdjusted(requested, effective));
  effective.position.x -= 0.3;
  EXPECT_TRUE(scan_planner::goalPositionAdjusted(requested, effective));
  effective = requested;
  effective.position.z = 0.03;
  EXPECT_TRUE(scan_planner::goalPositionAdjusted(requested, effective));
  EXPECT_DOUBLE_EQ(requested.position.z, 0.15);
}

TEST(GoalFeedback, NumericalNoiseIsNotAnAdjustedGoal) {
  geometry_msgs::msg::Pose requested;
  auto effective = requested;
  effective.position.x = 1e-8;
  EXPECT_FALSE(scan_planner::goalPositionAdjusted(requested, effective));
}
