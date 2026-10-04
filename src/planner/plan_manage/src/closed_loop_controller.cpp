#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <tuple>
#include <chrono>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <gbplanner3_interfaces/msg/controller_command.hpp>
#include <gbplanner3_interfaces/msg/controller_ack.hpp>
#include <gbplanner3_interfaces/msg/scan_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"
#include <plan_manage/terminal_speed.h>

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    terminal_kp_yaw_ = declare_parameter<double>("terminal_kp_yaw", 0.8);
    terminal_max_yaw_rate_ = declare_parameter<double>("terminal_max_yaw_rate", 0.5);
    if (!(terminal_kp_yaw_ > 0.0) || !std::isfinite(terminal_kp_yaw_) ||
        !(terminal_max_yaw_rate_ > 0.0) || !std::isfinite(terminal_max_yaw_rate_))
      throw std::invalid_argument("Terminal yaw gain and rate must be finite and positive");
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    terminal_speed_limit_ = declare_parameter<bool>("terminal_speed_limit", false);
    terminal_deceleration_ = declare_parameter<double>("terminal_deceleration", 0.5);
    terminal_braking_delay_ = declare_parameter<double>("terminal_braking_delay", 0.25);
    terminal_stop_distance_ = declare_parameter<double>("terminal_stop_distance", 0.03);
    if (!std::isfinite(terminal_deceleration_) || terminal_deceleration_ <= 0.0 ||
        !std::isfinite(terminal_braking_delay_) || terminal_braking_delay_ < 0.0 ||
        !std::isfinite(terminal_stop_distance_) || terminal_stop_distance_ < 0.0 ||
        terminal_stop_distance_ >= finish_dist_)
      throw std::invalid_argument("Invalid terminal braking parameters");

    contract_mode_ = declare_parameter<bool>("contract_mode", false);
    allow_goal_handoff_ = declare_parameter<bool>("allow_goal_handoff", false);
    terminal_goal_yaw_ = declare_parameter<bool>("terminal_goal_yaw", false);
    yaw_tolerance_ = declare_parameter<double>("terminal_yaw_tolerance", 0.15);
    odom_timeout_ = declare_parameter<double>("contract_odom_timeout", 0.5);
    if (yaw_tolerance_ <= 0.0 || odom_timeout_ <= 0.0)
      throw std::invalid_argument("Contract tolerances must be positive");
    if (contract_mode_) {
      ack_pub_ = create_publisher<Ack>("planning/controller_ack", 10);
      command_sub_ = create_subscription<Command>("planning/controller_command", 10,
          std::bind(&ClosedLoopController::contractCallback, this, std::placeholders::_1));
    } else if (terminal_goal_yaw_) {
      goal_trajectory_sub_ = create_subscription<gbplanner3_interfaces::msg::ScanTrajectory>(
          "planning/goal_trajectory", 10,
          std::bind(&ClosedLoopController::goalTrajectoryCallback, this, std::placeholders::_1));
    } else bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(contract_mode_ ? "planning/controller_cmd_vel" : "cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  using Command = gbplanner3_interfaces::msg::ControllerCommand;
  using Ack = gbplanner3_interfaces::msg::ControllerAck;
  using Identity = std::tuple<uint64_t, uint64_t, uint64_t, uint64_t>;
  static Identity identity(const Command &msg) {
    return {msg.exploration_session_id, msg.goal_id, msg.planning_attempt_id, msg.trajectory_id};
  }
  void ack(const Command &msg, uint8_t status, const std::string &reason) {
    Ack result;
    result.header = msg.header;
    result.header.stamp = now();
    result.exploration_session_id = msg.exploration_session_id;
    result.goal_id = msg.goal_id;
    result.planning_attempt_id = msg.planning_attempt_id;
    result.trajectory_id = msg.trajectory_id;
    result.status = status;
    result.reason = reason;
    ack_pub_->publish(result);
  }
  void clearContract() {
    receive_traj_ = false;
    traj_.clear();
    exec_time_ = 0.0;
    publishStop();  // ACK below means this zero was published, not physical standstill.
    publishExecutionFrozen(false);
    contract_active_ = false;
  }
  bool freshOdom() const {
    if (!have_odom_) return false;
    const double receipt_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - odom_receipt_).count();
    return receipt_age <= odom_timeout_;
  }
  void contractCallback(const Command::ConstSharedPtr msg) {
    const auto key = identity(*msg);
    if (!msg->exploration_session_id || !msg->goal_id || !msg->planning_attempt_id || !msg->trajectory_id) {
      ack(*msg, Ack::REJECTED, "INVALID_IDENTITY"); return;
    }
    if (msg->action == Command::CANCEL) {
      if (closed_.size() >= 4096 && !closed_.count(key) &&
          !(contract_active_ && identity(active_) == key)) {
        ack(*msg, Ack::REJECTED, "IDENTITY_CACHE_FULL_RESTART_REQUIRED"); return;
      }
      closed_.insert(key);  // A cancel arriving before START also prevents late activation.
      if (contract_active_ && identity(active_) == key) clearContract();
      else if (!contract_active_) publishStop();
      ack(*msg, Ack::CANCELLED, "IDENTITY_RETIRED_TRAJECTORY_CLEARED_NOT_PHYSICAL_STOP");
      return;
    }
    if (msg->action != Command::START || closed_.count(key)) {
      ack(*msg, Ack::REJECTED, "RETIRED_OR_INVALID_COMMAND"); return;
    }
    const bool replacing = contract_active_ && identity(active_) != key;
    if (contract_active_ && !replacing) {
      ack(*msg, Ack::ACCEPTED, "ALREADY_ACCEPTED_NO_RESTART");
      return;
    }
    const bool same_attempt = replacing &&
        msg->exploration_session_id == active_.exploration_session_id &&
        msg->goal_id == active_.goal_id &&
        msg->planning_attempt_id == active_.planning_attempt_id;
    const bool newer_goal = replacing && allow_goal_handoff_ &&
        msg->exploration_session_id == active_.exploration_session_id &&
        msg->goal_id > active_.goal_id && msg->planning_attempt_id > active_.planning_attempt_id;
    if (replacing && (!(same_attempt || newer_goal) || msg->trajectory_id <= active_.trajectory_id ||
        rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()) <=
        rclcpp::Time(active_.header.stamp, get_clock()->get_clock_type()))) {
      if (closed_.size() < 4096) closed_.insert(key);
      ack(*msg, Ack::REJECTED, "BUSY_CANCEL_REQUIRED_OR_OLD_TRAJECTORY");
      return;
    }
    if (closed_.size() >= 4096) {
      ack(*msg, Ack::REJECTED, "IDENTITY_CACHE_FULL_RESTART_REQUIRED"); return;
    }
    const auto &b = msg->trajectory;
    const auto &q = msg->target_pose.orientation;
    const auto &p = msg->target_pose.position;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    bool valid = b.traj_id > 0 && static_cast<uint64_t>(b.traj_id) == msg->trajectory_id &&
        b.order >= 2 && b.pos_pts.size() > static_cast<size_t>(b.order) &&
        b.knots.size() == b.pos_pts.size() + static_cast<size_t>(b.order) + 1 &&
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(qnorm) && std::abs(qnorm - 1.0) < 0.01;
    for (const auto &point : b.pos_pts)
      valid = valid && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    for (size_t i=0; i<b.knots.size(); ++i)
      valid = valid && std::isfinite(b.knots[i]) && (i == 0 || b.knots[i] > b.knots[i-1]);
    if (!valid || !freshOdom() || msg->header.frame_id.empty() || msg->header.frame_id != odom_frame_ ||
        now() >= rclcpp::Time(msg->valid_until, get_clock()->get_clock_type())) {
      closed_.insert(key);
      ack(*msg, Ack::REJECTED, "INVALID_TRAJECTORY_FRAME_LEASE_OR_ODOMETRY"); return;
    }
    // Install the validated replacement atomically: no zero command or stop wait.
    // Retired identities cannot cancel the newly installed goal.
    if (replacing) closed_.insert(identity(active_));
    active_ = *msg;
    contract_active_ = true;
    position_reported_ = false;
    pose_reported_ = false;
    bsplineCallback(std::make_shared<scan_planner_msgs::msg::Bspline>(b));
    ack(*msg, Ack::ACCEPTED, replacing ? "LIVE_TRAJECTORY_REPLACED" : "TRAJECTORY_INSTALLED");
  }

  static constexpr double kMaxVYawLimit = 2.0943951023931953;  // 120 deg/s

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void goalTrajectoryCallback(const gbplanner3_interfaces::msg::ScanTrajectory::ConstSharedPtr msg)
  {
    const auto &result = msg->result;
    const auto stamp = rclcpp::Time(result.header.stamp, get_clock()->get_clock_type());
    if (result.header.frame_id.empty() || result.header.frame_id != odom_frame_ ||
        stamp < last_goal_trajectory_stamp_) return;
    last_goal_trajectory_stamp_ = stamp;
    manual_terminal_active_ = false;
    manual_yaw_reported_ = false;
    bsplineCallback(std::make_shared<scan_planner_msgs::msg::Bspline>(msg->trajectory));
    if (!receive_traj_ || result.status != result.READY || !result.effective_goal_valid) return;
    const auto &q = result.requested_goal.orientation;
    const double qnorm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const auto &p = result.effective_goal.position;
    if (!std::isfinite(qnorm) || std::abs(qnorm-1.0) > .01 ||
        !std::isfinite(p.x) || !std::isfinite(p.y)) return;
    const auto end = traj_[0].evaluateDeBoorT(traj_duration_);
    // A local horizon endpoint is not the final viewpoint.
    manual_terminal_active_ = std::hypot(end.x()-p.x, end.y()-p.y) <= finish_dist_;
    manual_terminal_position_ = Eigen::Vector2d(p.x, p.y);
    manual_terminal_yaw_ = tf2::getYaw(result.requested_goal.orientation);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    if (!have_odom_ || stamp > odom_stamp_)
      odom_receipt_ = std::chrono::steady_clock::now();
    odom_stamp_ = stamp;
    odom_frame_ = msg->header.frame_id;
    have_odom_ = std::isfinite(odom_yaw_) && odom_pos_.allFinite();
  }

  bool nativeTrajectoryFinished()
  {
    if (!receive_traj_ || !have_odom_ || exec_time_ < traj_duration_) return false;
    const Eigen::Vector3d end = traj_[0].evaluateDeBoorT(traj_duration_);
    return (end.head<2>() - odom_pos_.head<2>()).norm() < finish_dist_;
  }

  void cmdCallback()
  {
    if (contract_mode_) {
      if (!contract_active_) return;
      if (!freshOdom() || odom_frame_ != active_.header.frame_id ||
          now() >= rclcpp::Time(active_.valid_until, get_clock()->get_clock_type())) {
        const char *reason = !freshOdom() ? "ODOMETRY_RECEIPT_TIMEOUT_TRAJECTORY_CLEARED" :
            (odom_frame_ != active_.header.frame_id ? "ODOMETRY_FRAME_MISMATCH_TRAJECTORY_CLEARED" :
             "LEASE_EXPIRED_TRAJECTORY_CLEARED");
        closed_.insert(identity(active_));
        clearContract();
        ack(active_, Ack::ABORTED, reason);
        return;
      }
      const double distance = std::hypot(active_.target_pose.position.x - odom_pos_.x(),
                                          active_.target_pose.position.y - odom_pos_.y());
      // Wait for native trajectory completion before terminal yaw/feedback.
      if (nativeTrajectoryFinished() && distance < finish_dist_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s goal=%llu attempt=%llu trajectory=%llu distance=%.4f yaw_error=%.4f require_yaw=%d",
            active_.require_terminal_yaw ? "TERMINAL_YAW" : "POSITION_REACHED_WAIT_SUCCESSOR",
            static_cast<unsigned long long>(active_.goal_id),
            static_cast<unsigned long long>(active_.planning_attempt_id),
            static_cast<unsigned long long>(active_.trajectory_id), distance,
            normalizeAngle(tf2::getYaw(active_.target_pose.orientation)-odom_yaw_),
            active_.require_terminal_yaw);
        if (!position_reported_) {
          publishStop();
          ack(active_, Ack::POSITION_REACHED, "MEASURED_POSITION_WITHIN_TOLERANCE");
          position_reported_ = true;
        }
        const double error = normalizeAngle(tf2::getYaw(active_.target_pose.orientation) - odom_yaw_);
        const bool aligned = !active_.require_terminal_yaw || std::abs(error) <= yaw_tolerance_;
        publishExecutionFrozen(true);
        publishStop(aligned ? 0.0 : std::clamp(terminal_kp_yaw_ * error,
            -terminal_max_yaw_rate_, terminal_max_yaw_rate_));
        last_update_time_ = now();
        if (aligned && !pose_reported_) {
          ack(active_, Ack::POSE_REACHED, "MEASURED_POSE_WITHIN_TOLERANCE_ZERO_PUBLISHED");
          pose_reported_ = true;
        }
        return;
      }
      position_reported_ = false;
      pose_reported_ = false;
    }
    if (!contract_mode_ && terminal_goal_yaw_) {
      if (!freshOdom()) {
        publishStop();
        publishExecutionFrozen(true);
        last_update_time_ = now();
        return;
      }
      if (nativeTrajectoryFinished() && manual_terminal_active_ &&
          (manual_terminal_position_ - odom_pos_.head<2>()).norm() <= finish_dist_) {
        const double error = normalizeAngle(manual_terminal_yaw_ - odom_yaw_);
        const bool aligned = std::abs(error) <= yaw_tolerance_;
        publishStop(aligned ? 0.0 : std::clamp(terminal_kp_yaw_ * error,
            -terminal_max_yaw_rate_, terminal_max_yaw_rate_));
        publishExecutionFrozen(true);
        last_update_time_ = now();
        if (aligned && !manual_yaw_reported_) {
          RCLCPP_INFO(get_logger(), "MANUAL_GOAL_POSE_REACHED yaw_error=%.3f", error);
          manual_yaw_reported_ = true;
        }
        return;
      }
    }
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    double desired_yaw = estimateDesiredYaw(t_eval, pos_des);
    // At the spline end, correct residual position with the nearest heading.
    // A target behind the rover uses reverse motion rather than a U-turn.
    if (contract_mode_ && exec_time_ >= traj_duration_ &&
        (pos_des.head<2>() - odom_pos_.head<2>()).norm() >= finish_dist_) {
      const Eigen::Vector2d error = pos_des.head<2>() - odom_pos_.head<2>();
      const double bearing = std::atan2(error.y(), error.x());
      const bool reverse = std::abs(normalizeAngle(bearing-odom_yaw_)) > M_PI/2.0;
      const double heading_error = normalizeAngle(bearing + (reverse ? M_PI : 0.0) - odom_yaw_);
      geometry_msgs::msg::Twist recovery;
      recovery.angular.z = std::clamp(terminal_kp_yaw_ * heading_error,
          -std::min(terminal_max_yaw_rate_, max_vyaw_),
           std::min(terminal_max_yaw_rate_, max_vyaw_));
      if (std::abs(heading_error) <= heading_error_threshold_) {
        const double longitudinal = std::cos(odom_yaw_)*error.x() + std::sin(odom_yaw_)*error.y();
        recovery.linear.x = std::clamp(kp_pos_ * longitudinal, -std::min(0.2, max_vx_), std::min(0.2, max_vx_));
      }
      // No spline-end velocity feedforward or lateral command during recovery.
      publishExecutionFrozen(true);
      cmd_vel_pub_->publish(recovery);
      last_update_time_ = current_time;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "TERMINAL_POSITION_RECOVERY distance=%.4f heading_error=%.4f reverse=%d vx=%.4f",
          error.norm(), heading_error, reverse, recovery.linear.x);
      return;
    }
    const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    double speed_limit = std::max(max_vx_, max_vy_);
    // Only brake for an identified final goal, never a rolling-horizon endpoint.
    // Recompute from measured distance on every tick, including after replanning.
    Eigen::Vector2d final_goal = manual_terminal_position_;
    bool final_segment = !contract_mode_ && terminal_goal_yaw_ && manual_terminal_active_;
    if (contract_mode_ && contract_active_) {
      final_goal = Eigen::Vector2d(active_.target_pose.position.x, active_.target_pose.position.y);
      final_segment = (traj_[0].evaluateDeBoorT(traj_duration_).head<2>() - final_goal).norm() <= finish_dist_;
    }
    if (terminal_speed_limit_ && final_segment) {
      speed_limit = std::min(speed_limit, terminalSpeedLimit(
          (final_goal - odom_pos_.head<2>()).norm(), terminal_deceleration_,
          terminal_braking_delay_, terminal_stop_distance_));
    }
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        speed_limit);
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool contract_mode_{false}, contract_active_{false};
  bool allow_goal_handoff_{false};
  bool terminal_goal_yaw_{false}, manual_terminal_active_{false}, manual_yaw_reported_{false};
  Eigen::Vector2d manual_terminal_position_{Eigen::Vector2d::Zero()};
  double manual_terminal_yaw_{0.0};
  rclcpp::Time last_goal_trajectory_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<gbplanner3_interfaces::msg::ScanTrajectory>::SharedPtr goal_trajectory_sub_;
  bool position_reported_{false}, pose_reported_{false};
  double yaw_tolerance_{0.15}, odom_timeout_{0.5};
  Command active_;
  std::set<Identity> closed_;
  rclcpp::Publisher<Ack>::SharedPtr ack_pub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  std::string odom_frame_;
  std::chrono::steady_clock::time_point odom_receipt_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_;
  double kp_pos_, kp_yaw_;
  double terminal_kp_yaw_, terminal_max_yaw_rate_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  bool terminal_speed_limit_{false};
  double terminal_deceleration_, terminal_braking_delay_, terminal_stop_distance_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
