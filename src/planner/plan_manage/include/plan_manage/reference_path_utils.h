#pragma once
#include <algorithm>
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <iostream>
#include <plan_manage/polynomial_initial_time.h>
namespace scan_planner {
// Preserve all waypoint geometry, but avoid near-zero min-snap segment times.
// The floor uses the existing B-spline control-point spacing and speed.
inline double referenceSegmentTime(double length, double speed, double spacing) {
  return std::max(length, spacing) / speed;
}
// Generate geometry-preserving min-snap references with a cruise interval.
// End-point derivatives remain unchanged when retiming; local collision and
// B-spline feasibility checks still apply to the final executable trajectory.
inline double referenceDerivativeBound(PolynomialTraj& curve, int derivative) {
  const auto durations = curve.getTimes();
  const auto x = curve.getCoef(0), y = curve.getCoef(1), z = curve.getCoef(2);
  auto choose = [](int n, int k) {
    double c = 1.;
    for (int i = 1; i <= k; ++i) c *= double(n-i+1)/i;
    return c;
  };
  double bound = 0.;
  for (size_t s = 0; s < durations.size(); ++s) {
    const int degree = int(x[s].size())-1, order = degree-derivative;
    std::vector<Eigen::Vector3d> power(order+1), bernstein(order+1, Eigen::Vector3d::Zero());
    for (int k = 0; k <= order; ++k) {
      double factor = std::pow(durations[s], k);
      for (int j = 1; j <= derivative; ++j) factor *= k+j;
      const int index = degree-k-derivative;
      power[k] = factor * Eigen::Vector3d(x[s][index],y[s][index],z[s][index]);
      if (!power[k].allFinite()) return std::numeric_limits<double>::infinity();
    }
    for (int i = 0; i <= order; ++i)
      for (int k = 0; k <= i; ++k) bernstein[i] += choose(i,k)/choose(order,k)*power[k];
    bound = std::max(bound, derivativeHullBound(bernstein));
  }
  return bound;
}

inline bool makeTimedReference(const std::vector<Eigen::Vector3d>& input,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc,
    double vmax, double amax, double spacing, PolynomialTraj& output) {
  if (input.size()<2 || !std::isfinite(vmax) || !std::isfinite(amax) ||
      !std::isfinite(spacing) || vmax<=0 || amax<=0 || spacing<=0 ||
      !start_vel.allFinite() || !end_vel.allFinite() ||
      !start_acc.allFinite() || !end_acc.allFinite()) return false;
  // An already excessive boundary state cannot be repaired by lengthening time.
  if (start_vel.norm()>vmax+1e-6 || end_vel.norm()>vmax+1e-6 ||
      start_acc.norm()>amax+1e-6 || end_acc.norm()>amax+1e-6) return false;
  std::vector<Eigen::Vector3d> waypoints;
  double length=0.;
  for (const auto& p : input) {
    if (!p.allFinite()) return false;
    if (waypoints.empty() || (p-waypoints.back()).norm()>1e-3) {
      if (!waypoints.empty()) length+=(p-waypoints.back()).norm();
      waypoints.push_back(p);
    }
  }
  if (waypoints.size()<2) return false;
  // Cruise knots prevent a long two-segment trajectory from behaving like one
  // slow rest-to-rest quintic. Bound added knots for long reference routes.
  const double step=std::max(length/64., std::clamp(vmax*vmax/amax, .5, 4.));
  std::vector<Eigen::Vector3d> points{waypoints.front()};
  for (size_t i=1;i<waypoints.size();++i) {
    const double distance=(waypoints[i]-waypoints[i-1]).norm();
    const int n=std::max(1,int(std::ceil(distance/step)));
    for (int j=1;j<=n;++j) points.push_back(waypoints[i-1]+(waypoints[i]-waypoints[i-1])*double(j)/n);
  }
  const int count=points.size();
  Eigen::MatrixXd positions(3,count);
  Eigen::VectorXd lengths(count-1), times(count-1), speeds=Eigen::VectorXd::Constant(count,vmax);
  for(int i=0;i<count;++i) positions.col(i)=points[i];
  for(int i=0;i<count-1;++i) lengths(i)=(points[i+1]-points[i]).norm();
  speeds(0)=std::clamp(start_vel.dot((points[1]-points[0]).normalized()),0.,vmax);
  speeds(count-1)=std::clamp(end_vel.dot((points.back()-points[count-2]).normalized()),0.,vmax);
  for(int i=1;i<count;++i) speeds(i)=std::min(speeds(i),std::sqrt(speeds(i-1)*speeds(i-1)+2*amax*lengths(i-1)));
  for(int i=count-2;i>=0;--i) speeds(i)=std::min(speeds(i),std::sqrt(speeds(i+1)*speeds(i+1)+2*amax*lengths(i)));
  for(int i=0;i<count-1;++i) {
    const double v0=speeds(i), v1=speeds(i+1);
    const double peak=std::min(vmax,std::sqrt(amax*lengths(i)+(v0*v0+v1*v1)*.5));
    const double ramp_distance=(2*peak*peak-v0*v0-v1*v1)/(2*amax);
    const double duration=(2*peak-v0-v1)/amax+std::max(0.,lengths(i)-ramp_distance)/peak;
    times(i)=std::max(referenceSegmentTime(lengths(i),vmax,spacing),duration);
  }
  // The speed envelope initializes time, but min-snap can overshoot it. Check
  // derivative convex hulls (not only sampled extrema), then rebuild with the
  // original boundary derivatives. No speed/acceleration limit is relaxed.
  for(int attempt=0;attempt<16;++attempt) {
    auto curve = count>2 ? PolynomialTraj::minSnapTraj(positions,start_vel,end_vel,start_acc,end_acc,times)
        : PolynomialTraj::one_segment_traj_gen(points.front(),start_vel,start_acc,points.back(),end_vel,end_acc,times(0));
    curve.init();
    const double v=referenceDerivativeBound(curve,1), a=referenceDerivativeBound(curve,2);
    if (!std::isfinite(v) || !std::isfinite(a)) return false;
    if (v<=vmax+1e-6 && a<=amax+1e-6) {output=curve;return true;}
    times *= std::clamp(1.01*std::max(v/vmax,std::sqrt(a/amax)),1.05,2.);
  }
  return false;
}
// The global polynomial is a geometric/time guide, not the executable spline.
// A native spline may use the existing feasibility tolerances. Its derivatives
// must remain intact for the local optimizer, but need not be boundary constraints
// of the nominal-speed guide. Never extend the executable admissible envelope.
inline bool makePlannerGuidanceReference(const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc,
    double vmax, double amax, double spacing, double admissible_v, double admissible_a,
    PolynomialTraj& output, const char*& reason) {
  reason = "INVALID_BOUNDARY_OR_LIMITS";
  if (!start_vel.allFinite() || !start_acc.allFinite() || !end_vel.allFinite() || !end_acc.allFinite()
      || !std::isfinite(admissible_v) || !std::isfinite(admissible_a)
      || !std::isfinite(vmax) || !std::isfinite(amax) || vmax<=0 || amax<=0
      || admissible_v<vmax || admissible_a<amax) return false;
  reason = "BOUNDARY_OUTSIDE_LOCAL_LIMITS";
  if (start_vel.norm()>admissible_v+1e-6 || end_vel.norm()>admissible_v+1e-6
      || start_acc.norm()>admissible_a+1e-6 || end_acc.norm()>admissible_a+1e-6) return false;
  if (makeTimedReference(points,start_vel,start_acc,end_vel,end_acc,vmax,amax,spacing,output)) {
    reason = "ORIGINAL_BOUNDARY"; return true;
  }
  const auto guide_velocity = [vmax](const Eigen::Vector3d& v) -> Eigen::Vector3d {
    return v.norm()>vmax ? v*(vmax/v.norm()) : v;
  };
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  if (makeTimedReference(points,guide_velocity(start_vel),zero,guide_velocity(end_vel),zero,
                         vmax,amax,spacing,output)) {
    reason = "NORMALIZED_GUIDE_BOUNDARY"; return true;
  }
  // A sharp handoff turn can make even the tangent-constrained guide infeasible.
  // Keep all ordered geometry knots; actual local start v/a are still supplied
  // separately by reboundReplan and checked after optimization as before.
  if (makeTimedReference(points,zero,zero,zero,zero,vmax,amax,spacing,output)) {
    reason = "REST_GUIDE_BOUNDARY"; return true;
  }
  reason = "GUIDE_TIME_ALLOCATION_FAILED";
  return false;
}

inline bool referenceTargetInRange(double distance, double horizon, double minimum) {
  return std::isfinite(distance) && distance >= minimum && distance <= horizon;
}
}
