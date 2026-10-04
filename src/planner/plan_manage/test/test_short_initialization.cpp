// Offline diagnostic only. No ROS node, map, publishers or hardware output.
#include <gtest/gtest.h>
#include <traj_utils/polynomial_traj.h>
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <iomanip>
#include <iostream>

TEST(ShortInitialization, AuditExistingPolynomialAndSplineInitialization) {
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  for (double distance : {0.21,0.22,0.30,0.50,1.0}) {
    const Eigen::Vector3d start(0,0,0), goal(distance,0,0);
    const double max_vel=1.,max_acc=1.,spacing=.2;
    const double duration=std::pow(max_vel,2)/max_acc>distance ?
        std::sqrt(distance/max_acc) :
        (distance-std::pow(max_vel,2)/max_acc)/max_vel+2*max_vel/max_acc;
    auto poly=PolynomialTraj::one_segment_traj_gen(start,zero,zero,goal,zero,zero,duration);
    EXPECT_LT((poly.evaluate(duration)-goal).norm(),1e-9);
    EXPECT_LT(poly.evaluateAcc(duration).norm(),1e-8);
    double peak=0;
    for(int i=0;i<=10000;++i) peak=std::max(peak,poly.evaluateAcc(duration*i/10000.).norm());
    // Exact quintic rest-to-rest peak: (10/sqrt(3))*distance/duration^2.
    EXPECT_NEAR(peak,10/std::sqrt(3.)*distance/(duration*duration),1e-5);
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
    std::vector<Eigen::Vector3d> deriv{poly.evaluateVel(0),zero,poly.evaluateAcc(0),poly.evaluateAcc(t)};
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
  }
}
