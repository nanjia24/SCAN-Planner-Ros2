// Offline synthetic FREE-map fixture. Isolated ROS domain, no executor or controller.
#include <gtest/gtest.h>
#include <plan_manage/polynomial_initial_time.h>
#include <bspline_opt/bspline_optimizer.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <traj_utils/polynomial_traj.h>
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <iomanip>
#include <iostream>

TEST(ShortOptimization, AuditActualOptimizerStagesOnSyntheticFreeMap) {
  const char* config=std::getenv("SCAN_AUDIT_CONFIG"); ASSERT_NE(config,nullptr);
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("scan_planner_node",rclcpp::NodeOptions().arguments({"--ros-args","--params-file",config}).parameter_overrides({
    rclcpp::Parameter("grid_map.sliding_map_size_x",3.0),
    rclcpp::Parameter("grid_map.sliding_map_size_y",3.0),
    rclcpp::Parameter("grid_map.sliding_map_size_z",2.0),
    rclcpp::Parameter("grid_map.ground_height",-1.0)}));
  auto map=std::make_shared<GridMap>();map->initMap(node.get());
  Eigen::Vector3d origin,size;map->getRegion(origin,size);double res=map->getResolution();
  for(double x=origin.x()+res/2;x<origin.x()+size.x();x+=res)
    for(double y=origin.y()+res/2;y<origin.y()+size.y();y+=res)
      for(double z=origin.z()+res/2;z<origin.z()+size.z();z+=res)
        map->setOccupancy(Eigen::Vector3d(x,y,z),0);
  Eigen::Vector3i origin_index; map->posToIndex(Eigen::Vector3d(0.,0.,0.),origin_index); ASSERT_TRUE(map->isKnownFree(origin_index));
  Eigen::Vector3d requested_goal=Eigen::Vector3d::Zero();
  auto report=[&](const char* label,scan_planner::UniformBspline& b){
    auto a=b.getDerivative().getDerivative();double peak=0,end=b.getTimeSum();
    for(int i=0;i<=10000;++i)peak=std::max(peak,a.evaluateDeBoorT(end*i/10000.).norm());
    std::cout<<"STAGE_AUDIT stage="<<label<<" duration="<<end<<" peak="<<peak
      <<" end_acc="<<a.evaluateDeBoorT(end).norm()
      <<" goal_error="<<(b.evaluateDeBoorT(end)-requested_goal).norm()
      <<" end_vel="<<b.getDerivative().evaluateDeBoorT(end).norm()<<std::endl;
    return peak;
  };
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  struct Case {double distance, velocity, acceleration;};
  std::vector<Case> cases{{.21,0,0},{.22,0,0},{.30,0,0},{.50,0,0},{1.,0,0}};
  if(std::getenv("SCAN_AUDIT_MOVING_CASES"))
    cases={{.22,.1,0},{.22,.3,0},{.22,.5,0},{-.22,0,0},
           {-.22,.1,0},{-.22,.3,0},{.22,.3,.5},{1.,.3,0}};
  for (const auto& c : cases) {
    const double distance=std::abs(c.distance);
    const Eigen::Vector3d start(0,0,0), goal(c.distance,0,0);
    const Eigen::Vector3d start_vel(c.velocity,0,0), start_acc(c.acceleration,0,0);
    requested_goal=goal;
    std::cout<<"CASE signed_distance="<<c.distance<<" start_vel="<<c.velocity
             <<" start_acc="<<c.acceleration<<std::endl;
    const double max_vel=1.,max_acc=1.,spacing=.2;
    double duration=std::pow(max_vel,2)/max_acc>distance ?
        std::sqrt(distance/max_acc) :
        (distance-std::pow(max_vel,2)/max_acc)/max_vel+2*max_vel/max_acc;
    // Rest-to-rest quintic analytical peaks: vmax=1.875*d/T,
    // amax=(10/sqrt(3))*d/T^2. Diagnostic only; not a general moving-start rule.
    if (std::getenv("SCAN_AUDIT_QUINTIC_TIME")) {
      duration=std::max(1.875*distance/max_vel,
                        std::sqrt((10/std::sqrt(3.))*distance/max_acc));
      std::cout<<"VARIANT analytical_quintic_time"<<std::endl;
    }
    if (std::getenv("SCAN_AUDIT_STATE_TIME")) {
      ASSERT_TRUE(scan_planner::selectPolynomialInitialTime(
          start,start_vel,start_acc,goal,zero,max_vel,max_acc,duration));
    }
    auto poly=PolynomialTraj::one_segment_traj_gen(start,start_vel,start_acc,goal,zero,zero,duration);
    EXPECT_LT((poly.evaluate(duration)-goal).norm(),1e-9);
    EXPECT_LT(poly.evaluateAcc(duration).norm(),1e-8);
    double peak=0;
    for(int i=0;i<=10000;++i) peak=std::max(peak,poly.evaluateAcc(duration*i/10000.).norm());
    // Exact quintic rest-to-rest peak: (10/sqrt(3))*distance/duration^2.
    if(c.velocity==0 && c.acceleration==0) {
      EXPECT_NEAR(peak,10/std::sqrt(3.)*distance/(duration*duration),1e-5);
    }
    double ts=spacing/max_vel*1.2;ts*=1.5;
    double t=0;bool too_far;std::vector<Eigen::Vector3d> points;
    do {
      ts/=1.5;points.clear();too_far=false;auto last=poly.evaluate(0);
      for(t=0;t<duration;t+=ts){
        auto p=poly.evaluate(t);
        if((last-p).norm()>spacing*1.5){too_far=true;break;}
        last=p;points.push_back(p);
      }
    }while(too_far || points.size()<7);
    t-=ts;
    if (std::getenv("SCAN_AUDIT_COMPLETE_ENDPOINT")) {
      ts=spacing/max_vel*1.2; // Same input interval as production, before legacy sampling.
      scan_planner::samplePolynomialEndpoints(poly,duration,spacing,ts,points);
      EXPECT_LT((points.front()-start).norm(),1e-9);
      EXPECT_LT((points.back()-goal).norm(),1e-9);
      t=duration;
    }
    std::vector<Eigen::Vector3d> deriv{poly.evaluateVel(0),zero,poly.evaluateAcc(0),poly.evaluateAcc(t)};
    // Diagnostic A/B only: preserve all points/times, replace terminal derivative.
    if (std::getenv("SCAN_AUDIT_ZERO_END_ACC")) {
      deriv.back()=poly.evaluateAcc(duration);
      std::cout<<"VARIANT zero_terminal_acc"<<std::endl;
    }
    Eigen::MatrixXd control;
    scan_planner::UniformBspline::parameterizeToBspline(ts,points,deriv,control);
    scan_planner::UniformBspline spline(control,3,ts);
    auto acc=spline.getDerivative().getDerivative();double spline_peak=0;
    for(int i=0;i<=10000;++i)spline_peak=std::max(spline_peak,acc.evaluateDeBoorT(spline.getTimeSum()*i/10000.).norm());
    ASSERT_TRUE(std::isfinite(spline_peak));
    std::cout<<std::setprecision(10)<<"INIT_AUDIT distance="<<distance<<" duration="<<duration
      <<" polynomial_peak="<<peak<<" sample_dt="<<ts<<" samples="<<points.size()
      <<" last_sample="<<t<<" terminal_acc_used="<<deriv.back().norm()
      <<" exact_terminal_acc="<<poly.evaluateAcc(duration).norm()
      <<" spline_duration="<<spline.getTimeSum()<<" spline_peak="<<spline_peak<<std::endl;
    scan_planner::BsplineOptimizer optimizer;
    optimizer.setParam(node.get());optimizer.setEnvironment(map);
    optimizer.a_star_.reset(new AStar);
    optimizer.a_star_->initGridMap(map,Eigen::Vector3i(100,100,100));
    optimizer.initControlPoints(control,true);
    ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(control,ts));
    scan_planner::UniformBspline after(control,3,ts);report("rebound",after);
    after.setPhysicalLimits(1.,1.,.5);double ratio=1.;
    bool feasible=after.checkFeasibility(ratio,false);
    std::cout<<"REALLOCATION feasible="<<feasible<<" ratio="<<ratio<<std::endl;
    if(!feasible){
      int segments=after.getControlPoint().cols()-3;
      after.lengthenTime(ratio);report("time_stretched_before_refit",after);
      double duration2=after.getTimeSum();ts=duration2/segments;
      std::vector<Eigen::Vector3d> resampled;
      for(double time=0;time<=duration2+1e-4;time+=ts)resampled.push_back(after.evaluateDeBoorT(time));
      scan_planner::UniformBspline::parameterizeToBspline(ts,resampled,deriv,control);
      scan_planner::UniformBspline refit(control,3,ts);report("reparam_with_original_derivatives",refit);
      double step=refit.getTimeSum()/(control.cols()-3);optimizer.ref_pts_.clear();
      for(double time=0;time<refit.getTimeSum()+1e-4;time+=step)optimizer.ref_pts_.push_back(refit.evaluateDeBoorT(time));
      Eigen::MatrixXd result;
      bool success=optimizer.BsplineOptimizeTrajRefine(control,ts,result);
      std::cout<<"REFINE success="<<success<<std::endl;
      if(success){scan_planner::UniformBspline final(result,3,ts);report("refine_final",final);after=final;}
    }
    if(std::getenv("SCAN_AUDIT_STATE_TIME") && std::getenv("SCAN_AUDIT_COMPLETE_ENDPOINT")) {
      auto acceleration=after.getDerivative().getDerivative();
      for(int i=0;i<=10000;++i)
        EXPECT_LE(acceleration.evaluateDeBoorT(after.getTimeSum()*i/10000.).norm(),2.5);
      EXPECT_LT((after.evaluateDeBoorT(after.getTimeSum())-goal).norm(),.001);
    }

  }
  map.reset();node.reset();rclcpp::shutdown();
}

TEST(PolynomialInitialTime, RejectInvalidOrAlreadyExcessiveBoundary) {
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero(), goal(.22,0,0);
  double duration=.5;
  EXPECT_FALSE(scan_planner::selectPolynomialInitialTime(zero,Eigen::Vector3d(1.1,0,0),zero,goal,zero,1,1,duration));
  EXPECT_FALSE(scan_planner::selectPolynomialInitialTime(zero,zero,Eigen::Vector3d(0,1.1,0),goal,zero,1,1,duration));
  EXPECT_FALSE(scan_planner::selectPolynomialInitialTime(zero,zero,zero,goal,zero,0,1,duration));
}

TEST(PolynomialInitialTime, MovingStatesRespectPolynomialBounds) {
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  for(const Eigen::Vector3d& goal : {Eigen::Vector3d(.22,0,0),Eigen::Vector3d(-.22,0,0),Eigen::Vector3d(.22,.22,0)}) {
    for(const Eigen::Vector3d& velocity : {zero,Eigen::Vector3d(.3,.1,0),Eigen::Vector3d(.8,0,0)}) {
      double duration=.45;
      ASSERT_TRUE(scan_planner::selectPolynomialInitialTime(zero,velocity,zero,goal,zero,1,1,duration));
      auto p=PolynomialTraj::one_segment_traj_gen(zero,velocity,zero,goal,zero,zero,duration);
      for(int i=0;i<=1000;++i){
        EXPECT_LE(p.evaluateVel(duration*i/1000.).norm(),1.+1e-8);
        EXPECT_LE(p.evaluateAcc(duration*i/1000.).norm(),1.+1e-8);
      }
      EXPECT_LT((p.evaluate(duration)-goal).norm(),1e-8);
    }
  }
}
