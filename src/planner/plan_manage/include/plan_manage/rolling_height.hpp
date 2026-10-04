#pragma once
#include <cmath>
#include <deque>
#include <utility>

namespace scan_planner {
// Low-frequency odometry only; timestamps are seconds in the ROS clock domain.
class RollingHeight {
public:
  void add(double stamp, double z, double now) {
    if (!std::isfinite(stamp) || !std::isfinite(z) || !std::isfinite(now) ||
        stamp <= 0 || now-stamp > .5 || stamp-now > .05) return;
    if (!samples_.empty() && stamp <= samples_.back().first) return;
    samples_.emplace_back(stamp,z);
    prune(now);
  }
  bool mean(double now, double &height) {
    prune(now);
    if (samples_.empty() || now-samples_.back().first > .5 || samples_.back().first-now > .05) return false;
    height = 0.;
    for (const auto &sample:samples_) height += sample.second;
    height /= samples_.size();
    return true;
  }
  bool updateNeeded(double now, double adopted, double &candidate) {
    return mean(now,candidate) && std::abs(candidate-adopted) > .10 + 1e-9;
  }
private:
  void prune(double now) {
    while (!samples_.empty() && now-samples_.front().first > 1.) samples_.pop_front();
  }
  std::deque<std::pair<double,double>> samples_;
};
}
