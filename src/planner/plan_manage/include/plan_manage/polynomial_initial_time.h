#pragma once

#include <Eigen/Core>
#include <traj_utils/polynomial_traj.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace scan_planner {
// Convex-hull upper bound for a Bezier curve. Subdivision tightens the bound;
// it does not sample away an extremum between evaluation times.
inline double derivativeHullBound(const std::vector<Eigen::Vector3d>& controls,
                                  unsigned depth = 4) {
  if (depth == 0) {
    double bound = 0;
    for (const auto& c : controls) bound = std::max(bound, c.norm());
    return bound;
  }
  auto work = controls;
  std::vector<Eigen::Vector3d> left(controls.size()), right(controls.size());
  left.front() = work.front(); right.back() = work.back();
  for (size_t level = 1; level < controls.size(); ++level) {
    for (size_t i = 0; i < controls.size() - level; ++i)
      work[i] = (work[i] + work[i + 1]) * 0.5;
    left[level] = work.front();
    right[controls.size() - level - 1] = work[controls.size() - level - 1];
  }
  return std::max(derivativeHullBound(left, depth - 1),
                  derivativeHullBound(right, depth - 1));
}

inline bool selectPolynomialInitialTime(
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& a0, const Eigen::Vector3d& p1,
    const Eigen::Vector3d& v1, double vmax, double amax, double& duration) {
  if (!p0.allFinite() || !v0.allFinite() || !a0.allFinite() ||
      !p1.allFinite() || !v1.allFinite() || !std::isfinite(vmax) ||
      !std::isfinite(amax) || vmax <= 0 || amax <= 0 ||
      !std::isfinite(duration) || duration <= 0 ||
      v0.norm() > vmax || v1.norm() > vmax || a0.norm() > amax) return false;
  // Only the initialization polynomial is rebuilt here. Existing B-spline
  // refinement and final feasibility checks remain responsible for its output.
  for (unsigned attempt = 0; attempt < 60; ++attempt) {
    std::vector<Eigen::Vector3d> p(6), v(5), a(4);
    p[0] = p0; p[1] = p0 + v0 * duration / 5.;
    p[2] = a0 * duration * duration / 20. + 2. * p[1] - p[0];
    p[5] = p1; p[4] = p1 - v1 * duration / 5.;
    p[3] = 2. * p[4] - p[5]; // Terminal acceleration is zero.
    for (size_t i = 0; i < v.size(); ++i) v[i] = 5. * (p[i+1]-p[i])/duration;
    for (size_t i = 0; i < a.size(); ++i) a[i] = 4. * (v[i+1]-v[i])/duration;
    if (derivativeHullBound(v) <= vmax && derivativeHullBound(a) <= amax)
      return true;
    duration *= 1.1;
  }
  return false;
}
// Include both endpoints without adding a short, nonuniform last interval.
inline void samplePolynomialEndpoints(PolynomialTraj& polynomial, double duration,
                                      double spacing, double& dt,
                                      std::vector<Eigen::Vector3d>& points) {
  int segments=std::max(6,static_cast<int>(std::ceil(duration/dt)));
  bool too_far;
  do {
    dt=duration/segments; points.clear(); too_far=false;
    for(int i=0;i<=segments;++i) {
      const auto p=polynomial.evaluate(i==segments ? duration : i*dt);
      if(!points.empty() && (p-points.back()).norm()>spacing*1.5) too_far=true;
      points.push_back(p);
    }
    if(too_far) segments*=2;
  } while(too_far);
}
}  // namespace scan_planner
