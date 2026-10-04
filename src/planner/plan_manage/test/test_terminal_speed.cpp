#include <gtest/gtest.h>
#include <plan_manage/terminal_speed.h>
TEST(TerminalSpeed, BrakingDistanceAndRecordedEndApproach) {
 double previous=0;
 for(double d=0; d<3; d+=.001) {
  double v=scan_planner::terminalSpeedLimit(d,.5,.25,.03);
  EXPECT_GE(v,previous); previous=v;
  EXPECT_LE(v*.25+v*v/(2*.5),std::max(0.,d-.03)+1e-12);
 }
 EXPECT_DOUBLE_EQ(scan_planner::terminalSpeedLimit(.02,.5,.25,.03),0);
 EXPECT_LT(scan_planner::terminalSpeedLimit(.205,.5,.25,.03),.32);
 EXPECT_GT(scan_planner::terminalSpeedLimit(2.,.5,.25,.03),1.);
 EXPECT_NEAR(scan_planner::terminalSpeedLimit(1.,.5,0.,0.),1.,1e-12);
}
TEST(TerminalSpeed, LaggedVehicleStopsWithinOneCentimeterOvershoot) {
 // Illustrative first-order velocity response, not a hardware validation.
 for(double response : {.1,.2,.25}) {
  double x=0,v=0;
  for(int i=0;i<20000;++i) {
   double command=std::min(1.,scan_planner::terminalSpeedLimit(3.-x,.5,.25,.03));
   v+=(command-v)*.001/response; x+=v*.001;
   EXPECT_LT(x,3.01);
  }
  // The .03 m zero-command zone reserves room for residual coasting.
  EXPECT_GE(x,2.96); EXPECT_LE(x,3.01); EXPECT_LT(v,.01);
 }
}
