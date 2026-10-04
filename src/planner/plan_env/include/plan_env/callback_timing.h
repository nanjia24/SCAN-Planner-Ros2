#pragma once
// Opt-in diagnostic timing; disabled unless SCAN_CALLBACK_TIMING=1.
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
namespace scan_diagnostics {
class CallbackTiming {
 public:
  CallbackTiming(rclcpp::Node* node, const char* label, int64_t stamp=0)
      : node_(node), label_(label), stamp_(stamp) {
    static const bool enabled = [] { const char* v=std::getenv("SCAN_CALLBACK_TIMING"); return v && std::string(v)=="1"; }();
    enabled_=enabled;
    if(enabled_) { start_=std::chrono::steady_clock::now(); clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu_); ros_start_=node_->now().nanoseconds(); }
  }
  ~CallbackTiming() {
    if(!enabled_) return;
    const auto end=std::chrono::steady_clock::now();
    timespec cpu; clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu);
    const double wall=std::chrono::duration<double,std::milli>(end-start_).count();
    const double thread=(cpu.tv_sec-cpu_.tv_sec)*1000.0+(cpu.tv_nsec-cpu_.tv_nsec)*1e-6;
    RCLCPP_INFO(node_->get_logger(),"SCAN_TIMING stage=%s stamp=%lld start=%lld wall_ms=%.3f cpu_ms=%.3f age_ms=%.3f",label_,static_cast<long long>(stamp_),static_cast<long long>(ros_start_),wall,thread,stamp_ ? (ros_start_-stamp_)*1e-6 : -1.0);
  }
 private:
  rclcpp::Node* node_; const char* label_; int64_t stamp_,ros_start_=0;
  bool enabled_=false; timespec cpu_{}; std::chrono::steady_clock::time_point start_;
};
}
