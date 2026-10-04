#include <gtest/gtest.h>
#include <traj_utils/polynomial_traj.h>
#include <plan_manage/reference_path_utils.h>
TEST(ReferencePath, RecordedMillimeterStartRemainsWellConditioned) {
std::vector<Eigen::Vector3d> p;
p.push_back(Eigen::Vector3d(-.435669,-1.538774,.026247));
p.push_back(Eigen::Vector3d(-0.43565126396300796,-1.5375340024973716,0.02636150530492761));
p.push_back(Eigen::Vector3d(0.03751987268034457,-1.3760123390482653,0.021908676510620125));
p.push_back(Eigen::Vector3d(0.5106910093236972,-1.214490675599159,0.017455847716312638));
p.push_back(Eigen::Vector3d(0.9838621459670496,-1.0529690121500526,0.013003018922005151));
p.push_back(Eigen::Vector3d(1.4570332826104022,-0.8914473487009461,0.008550190127697664));
p.push_back(Eigen::Vector3d(1.9302044192537546,-0.7299256852518398,0.00409736133339017));
p.push_back(Eigen::Vector3d(2.1431420993370702,-0.6572372948693661,0.002093487977981555));
p.push_back(Eigen::Vector3d(2.613709648358707,-0.48870720756318187,0.014891592538406807));
p.push_back(Eigen::Vector3d(3.0842771973803433,-0.3201771202569976,0.02768969709883206));
p.push_back(Eigen::Vector3d(3.55484474640198,-0.1516470329508134,0.04048780165925731));
p.push_back(Eigen::Vector3d(3.681167277785347,-0.10640560632239504,0.04392341673374175));
p.push_back(Eigen::Vector3d(3.781121615381225,-0.5533733881114844,-0.024972921609878553));

PolynomialTraj curve;
ASSERT_TRUE(scan_planner::makeTimedReference(p,Eigen::Vector3d(-.000603,.000856,-.00129),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),1.,1.,.2,curve));
curve.init();
for(double t=0;t<curve.getTimeSum();t+=.005) {
 auto point=curve.evaluate(t); ASSERT_TRUE(point.allFinite());
 EXPECT_LT(std::abs(point.z()),.1); EXPECT_LT(curve.evaluateVel(t).norm(),1.5);
}
EXPECT_LT((curve.evaluate(0)-p.front()).norm(),1e-6);
EXPECT_LT((curve.evaluate(curve.getTimeSum())-p.back()).norm(),1e-6);
}
TEST(ReferencePath, ReplacementRequiresProgressWithinHorizon) {
 EXPECT_FALSE(scan_planner::referenceTargetInRange(0.,4.,.1));
 EXPECT_FALSE(scan_planner::referenceTargetInRange(4.8,4.,.1));
 EXPECT_TRUE(scan_planner::referenceTargetInRange(2.,4.,.1));
}


TEST(ReferencePath, CruiseSpeedAndEndStateAcrossDistances) {
 const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
 for(double d : {.05,.21,.5,1.,2.,3.46,4.,7.7,9.84,20.}) {
  for(double initial_speed : {0.,.5,.9}) {
   // Short braking paths can be physically impossible without overshooting.
   if(d < initial_speed*initial_speed+.05) continue;
   PolynomialTraj curve;
   ASSERT_TRUE(scan_planner::makeTimedReference({zero,Eigen::Vector3d(d,0,0)},
     Eigen::Vector3d(initial_speed,0,0),zero,zero,zero,1.,1.,.2,curve)) << d << " " << initial_speed;
   double duration=curve.getTimeSum();
   if(d>=7.7) { EXPECT_GT(d/duration,.68) << d << " duration=" << duration; }
   EXPECT_LT((curve.evaluate(0)-zero).norm(),1e-6);
   EXPECT_LT((curve.evaluate(duration)-Eigen::Vector3d(d,0,0)).norm(),1e-6);
   EXPECT_NEAR(curve.evaluateVel(0).x(),initial_speed,1e-6);
   EXPECT_LT(curve.evaluateVel(duration).norm(),1e-6);
   EXPECT_LT(curve.evaluateAcc(duration).norm(),1e-6);
   for(int i=0;i<=1000;++i) {
    const double t=duration*i/1000.;
    EXPECT_LE(curve.evaluateVel(t).norm(),1.+1e-6);
    EXPECT_LE(curve.evaluateAcc(t).norm(),1.+1e-6);
    EXPECT_GE(curve.evaluate(t).x(),-1e-5);
    EXPECT_LE(curve.evaluate(t).x(),d+1e-5);
   }
   std::cout << "TIMING_AUDIT d=" << d << " v0=" << initial_speed << " duration=" << duration << " mean=" << d/duration << std::endl;
  }
 }
}

TEST(ReferencePath, OrderedBendAndBoundaryDerivativesArePreserved) {
 const Eigen::Vector3d zero=Eigen::Vector3d::Zero(), v(.3,0,0), a(.1,0,0);
 const std::vector<Eigen::Vector3d> points{zero,{3,0,0},{3,3,.2},{6,3,.2}};
 PolynomialTraj curve;
 ASSERT_TRUE(scan_planner::makeTimedReference(points,v,a,zero,zero,1.,1.,.2,curve));
 EXPECT_LT((curve.evaluateVel(0)-v).norm(),1e-6);
 EXPECT_LT((curve.evaluateAcc(0)-a).norm(),1e-6);
 // Every requested waypoint remains a polynomial knot, in order.
 double t=0;size_t reached=1;
 for(double dt:curve.getTimes()) {
  t+=dt;
  if(reached<points.size() && (curve.evaluate(t)-points[reached]).norm()<1e-5) ++reached;
 }
 EXPECT_EQ(reached,points.size());
 EXPECT_LE(scan_planner::referenceDerivativeBound(curve,1),1.+1e-6);
 EXPECT_LE(scan_planner::referenceDerivativeBound(curve,2),1.+1e-6);
}

TEST(ReferencePath, InvalidAndDuplicateInputs) {
 const Eigen::Vector3d z=Eigen::Vector3d::Zero();PolynomialTraj curve;
 EXPECT_FALSE(scan_planner::makeTimedReference({z,z},z,z,z,z,1,1,.2,curve));
 EXPECT_FALSE(scan_planner::makeTimedReference({z,{1,0,0}},z,z,z,z,0,1,.2,curve));
 EXPECT_FALSE(scan_planner::makeTimedReference({z,{NAN,0,0}},z,z,z,z,1,1,.2,curve));
 EXPECT_TRUE(scan_planner::makeTimedReference({z,z,{2,0,0}},z,z,z,z,1,1,.2,curve));
}

TEST(ReferencePath, GuideCanUseNominalBoundaryWithoutMutatingExecutableBoundary) {
 const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
 for (const Eigen::Vector3d& v : std::vector<Eigen::Vector3d>{{1.4,0,0},{0,1.1,0},{.7,0,0}}) {
  const Eigen::Vector3d a(1.6,0,0);
  PolynomialTraj guide;const char* reason=nullptr;
  ASSERT_TRUE(scan_planner::makePlannerGuidanceReference({zero,{8,0,0}},v,a,zero,zero,
      1.3,1.,.2,1.6,2.5,guide,reason));
  EXPECT_NE(std::string(reason),"ORIGINAL_BOUNDARY");
  EXPECT_LE(scan_planner::referenceDerivativeBound(guide,1),1.3+1e-6);
  EXPECT_LE(scan_planner::referenceDerivativeBound(guide,2),1.+1e-6);
  EXPECT_NEAR(a.x(),1.6,1e-12);
  EXPECT_LT((guide.evaluate(guide.getTimeSum())-Eigen::Vector3d(8,0,0)).norm(),1e-6);
 }
 PolynomialTraj rejected;const char* reason=nullptr;
 EXPECT_FALSE(scan_planner::makePlannerGuidanceReference({zero,{8,0,0}},{2,0,0},zero,zero,zero,
      1.3,1.,.2,1.6,2.5,rejected,reason));
 EXPECT_EQ(std::string(reason),"BOUNDARY_OUTSIDE_LOCAL_LIMITS");
 EXPECT_FALSE(scan_planner::makePlannerGuidanceReference({zero,{NAN,0,0}},zero,zero,zero,zero,
      1.3,1.,.2,1.6,2.5,rejected,reason));
}
