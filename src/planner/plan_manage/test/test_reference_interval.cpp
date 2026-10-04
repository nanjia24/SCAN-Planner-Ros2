#include <gtest/gtest.h>
#include <plan_manage/plan_container.hpp>
#include <limits>

namespace {
using scan_planner::GlobalTrajData;
using scan_planner::UniformBspline;
using Vec = Eigen::Vector3d;

GlobalTrajData straight()
{
  // A 12 m reference, while the local map only reaches about 5 m forward.
  PolynomialTraj p;
  p.addSegment({0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0}, 12.0);
  GlobalTrajData data;
  data.setGlobalTraj(p, rclcpp::Time(0));
  return data;
}

TEST(ReferenceInterval, RetractedTargetDoesNotLeaveOldHorizonSamples)
{
  auto data = straight();
  std::vector<Vec> old_points, old_derivatives, points, derivatives;
  double old_dt, old_duration, dt;
  data.getTrajByRadius(0, 7.5, .2, old_points, old_derivatives, old_dt, old_duration);
  old_points.back() = Vec(4.8, 0, 0);
  // Reproduce the old fold: the penultimate point is still near 7.5 m.
  ASSERT_GT(old_points[old_points.size()-2].x(), 7.0);
  ASSERT_GT(old_points[old_points.size()-2].x(), old_points.back().x());

  ASSERT_TRUE(data.getTrajBetweenTimes(0, 4.8, .2, points, derivatives, dt));
  EXPECT_GE(points.size(), 7u);
  EXPECT_NEAR(dt * (points.size()-1), 4.8, 1e-12);
  for (size_t i=0; i<points.size(); ++i) {
    EXPECT_GE(points[i].x(), 0.0);
    EXPECT_LE(points[i].x(), 4.8);
    if (i) { EXPECT_GT(points[i].x(), points[i-1].x()); }
  }
  Eigen::MatrixXd controls;
  UniformBspline::parameterizeToBspline(dt, points, derivatives, controls);
  UniformBspline spline(controls, 3, dt);
  for (int i=0; i<=500; ++i) {
    const double t=spline.getTimeSum()*i/500.;
    const auto p=spline.evaluateDeBoorT(t);
    EXPECT_GE(p.x(), -1e-8);
    EXPECT_LE(p.x(), 4.8+1e-8);
    EXPECT_NEAR(p.y(), 0, 1e-9);
    EXPECT_NEAR(spline.getDerivative().evaluateDeBoorT(t).x(), 1, 1e-8);
  }
}

TEST(ReferenceInterval, ProgressAndForwardAdjustedTargetsUseExactInterval)
{
  auto data=straight();
  for (double end : {3.0, 9.8, 12.0}) {
    std::vector<Vec> points, derivatives; double dt;
    ASSERT_TRUE(data.getTrajBetweenTimes(2, end, .2, points, derivatives, dt));
    EXPECT_NEAR(points.front().x(), 2, 1e-12);
    EXPECT_NEAR(points.back().x(), end, 1e-12);
    EXPECT_NEAR(dt*(points.size()-1), end-2, 1e-12);
    EXPECT_NEAR(derivatives[1].x(), 1, 1e-12);
  }
}

TEST(ReferenceInterval, OrderedCurveKeepsItsBendAndSelectedTime)
{
  // Curved path x=t*(4-t), y=t: x repeats, so nearest X/distance is ambiguous.
  PolynomialTraj p;
  p.addSegment({0, 0, 0, -1, 4, 0}, {0, 0, 0, 0, 1, 0},
               {0, 0, 0, 0, 0, 0}, 4);
  GlobalTrajData data; data.setGlobalTraj(p, rclcpp::Time(0));
  std::vector<Vec> points, derivatives; double dt;
  ASSERT_TRUE(data.getTrajBetweenTimes(1, 3, .2, points, derivatives, dt));
  EXPECT_NEAR((points.front()-Vec(3,1,0)).norm(),0,1e-10);
  EXPECT_NEAR((points.back()-Vec(3,3,0)).norm(),0,1e-10);
  double max_x=0;
  for (size_t i=0; i<points.size(); ++i) {
    const double t=1+2.0*i/(points.size()-1);
    EXPECT_NEAR((points[i]-Vec(t*(4-t),t,0)).norm(),0,1e-10);
    max_x=std::max(max_x,points[i].x());
  }
  EXPECT_GT(max_x,3.9);  // Must not shortcut the bend with a straight chord.
  EXPECT_NEAR((derivatives[1]-Vec(-2,1,0)).norm(),0,1e-10);
}

TEST(ReferenceInterval, ShortNonzeroIntervalHasFiniteEndpointSamples)
{
  auto data=straight();std::vector<Vec> points, derivatives;double dt;
  ASSERT_TRUE(data.getTrajBetweenTimes(1,1.001,.2,points,derivatives,dt));
  EXPECT_EQ(points.size(),7u); EXPECT_GT(dt,0);
  EXPECT_NEAR(points.back().x(),1.001,1e-12);
}

TEST(ReferenceInterval, InvalidIntervalClearsPreviousOutput)
{
  auto data=straight();
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for (const auto &args : std::vector<Vec>{{2,1,.2},{1,1,.2},{-1,2,.2},
           {1,13,.2},{nan,2,.2},{1,nan,.2},{1,2,0},{1,2,nan}}) {
    std::vector<Vec> points{Vec::Ones()},derivatives{Vec::Ones()};double dt=1;
    EXPECT_FALSE(data.getTrajBetweenTimes(args.x(),args.y(),args.z(),points,derivatives,dt));
    EXPECT_TRUE(points.empty());EXPECT_TRUE(derivatives.empty());EXPECT_EQ(dt,0);
  }
}
}  // namespace
