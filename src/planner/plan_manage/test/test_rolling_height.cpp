#include <gtest/gtest.h>
#include <plan_manage/rolling_height.hpp>
#include <limits>
using scan_planner::RollingHeight;
TEST(RollingHeight, WindowAndStrictTenCentimeterThreshold) {
  RollingHeight h; double z;
  h.add(10.,0.,10.); h.add(10.5,.2,10.5);
  ASSERT_TRUE(h.mean(10.5,z)); EXPECT_NEAR(z,.1,1e-12);
  EXPECT_FALSE(h.updateNeeded(10.5,0.,z));
  h.add(10.6,.3,10.6);
  EXPECT_TRUE(h.updateNeeded(10.6,0.,z)); EXPECT_NEAR(z,1./6.,1e-12);
  h.add(11.1,.3,11.1);
  ASSERT_TRUE(h.mean(11.1,z)); EXPECT_NEAR(z,.8/3.,1e-12); // oldest dropped
}
TEST(RollingHeight, StaleDuplicateInvalidAndNewSamples) {
  RollingHeight h; double z;
  EXPECT_FALSE(h.mean(10.,z));
  h.add(10.,-6.,10.); h.add(10.,100.,10.);
  h.add(9.,100.,10.); h.add(11.,100.,10.);
  h.add(10.1,std::numeric_limits<double>::quiet_NaN(),10.1);
  ASSERT_TRUE(h.mean(10.1,z)); EXPECT_DOUBLE_EQ(z,-6.);
  EXPECT_FALSE(h.updateNeeded(10.6,-5.,z));
  h.add(12.,-5.8,12.);
  ASSERT_TRUE(h.mean(12.,z)); EXPECT_DOUBLE_EQ(z,-5.8);
}
TEST(RollingHeight, SlowRiseComparedToAdoptedNotPreviousMean) {
  RollingHeight h; double candidate=0., adopted=0.;
  for (int i=0;i<=6;++i) {
    const double t=10.+i*.2;
    h.add(t,i*.03,t);
    if(i<6) { EXPECT_FALSE(h.updateNeeded(t,adopted,candidate)); }
  }
  EXPECT_TRUE(h.updateNeeded(11.2,adopted,candidate));
  adopted=candidate;
  EXPECT_FALSE(h.updateNeeded(11.2,adopted,candidate));
}
