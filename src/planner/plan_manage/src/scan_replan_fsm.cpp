#include <plan_manage/reference_path_utils.h>
#include "plan_env/callback_timing.h"

#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/goal_feedback.hpp>
#include <cmath>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    exploration_waypoint_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    exploration_reference_path_time_ =
        rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

    /*  fsm param  */
    contract_planning_only_ = load_parameter<bool>(node_, "fsm.contract_planning_only", false);
    contract_mode_ = contract_planning_only_ || load_parameter<bool>(node_, "fsm.contract_mode", false);
    allow_goal_handoff_ = load_parameter<bool>(node_, "fsm.allow_goal_handoff", false);
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    ground_height_follow_ = load_parameter<bool>(node_, "fsm.ground_height_follow", false);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    reference_segment_reached_tolerance_ = load_parameter<double>(
        node_, "fsm.reference_segment_reached_tolerance", 0.20);
    if (!std::isfinite(reference_segment_reached_tolerance_) || reference_segment_reached_tolerance_ <= 0)
      throw std::runtime_error("invalid reference segment reached tolerance");
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    exploration_direction_replan_threshold_ = load_parameter<double>(
        node_, "fsm.exploration_direction_replan_threshold", 0.5);
    exploration_waypoint_timeout_ = load_parameter<double>(
        node_, "fsm.exploration_waypoint_timeout", 3.0);
    exploration_return_stop_distance_ = load_parameter<double>(
        node_, "fsm.exploration_return_stop_distance", 0.5);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    goal_trajectory_pub_ = node_->create_publisher<gbplanner3_interfaces::msg::ScanTrajectory>("planning/goal_trajectory", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    fsm_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "planning/fsm_state", rclcpp::QoS(1).reliable().transient_local());
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String initial_fsm_state;
    initial_fsm_state.data = "INIT";
    fsm_state_pub_->publish(initial_fsm_state);

    if (contract_mode_)
    {
      if (navi_mode_ != NAVI_MODE::MANUAL_TARGET && navi_mode_ != NAVI_MODE::REFERENCE_PATH)
        throw std::runtime_error("contract mode requires navi_mode=1 or 3");
      contract_pub_ = node_->create_publisher<gbplanner3_interfaces::msg::PlanningResult>("planning/contract_result", 10);
      contract_trajectory_pub_ = node_->create_publisher<gbplanner3_interfaces::msg::ScanTrajectory>("planning/contract_trajectory", 10);
      contract_events_sub_ = node_->create_subscription<gbplanner3_interfaces::msg::ExecutionObservation>(
          "planning/execution_events", 10, std::bind(&SCANReplanFSM::contractExecutionCallback, this, std::placeholders::_1));
      contract_observation_sub_ = node_->create_subscription<gbplanner3_interfaces::msg::ExecutionObservation>(
          "planning/execution_observation", 10, std::bind(&SCANReplanFSM::contractExecutionCallback, this, std::placeholders::_1));
      contract_sub_ = node_->create_subscription<gbplanner3_interfaces::msg::LocalGoal>(
          "planning/local_goal", 10, std::bind(&SCANReplanFSM::contractGoalCallback, this, std::placeholders::_1));
    }
    else if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else if (navi_mode_ == NAVI_MODE::EXPLORATION)
    {
      exploration_waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
          "way_point", 5,
          std::bind(&SCANReplanFSM::explorationWaypointCallback, this, std::placeholders::_1));
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "exploration_reference_path", 1,
          std::bind(&SCANReplanFSM::explorationReferencePathCallback, this,
                    std::placeholders::_1));
      exploration_finish_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "exploration_finish", 5,
          std::bind(&SCANReplanFSM::explorationFinishCallback, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(),
                  "Exploration waypoint mode is ready; ordered reference path enabled");
    }
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, 3, or 4");
  }

  void SCANReplanFSM::contractResult(const gbplanner3_interfaces::msg::LocalGoal &goal,
      uint8_t status, const std::string &reason, uint64_t trajectory)
  {
    gbplanner3_interfaces::msg::PlanningResult result;
    result.header.stamp = node_->now();
    result.header.frame_id = goal.header.frame_id;
    result.exploration_session_id = goal.exploration_session_id;
    result.goal_id = goal.goal_id;
    result.planning_attempt_id = goal.planning_attempt_id;
    result.trajectory_id = trajectory;
    result.status = status;
    result.reason = reason;
    RCLCPP_INFO(node_->get_logger(),
        "SCAN_CONTRACT_RESULT session=%llu goal=%llu attempt=%llu trajectory=%llu status=%u reason=%s",
        static_cast<unsigned long long>(goal.exploration_session_id),
        static_cast<unsigned long long>(goal.goal_id),
        static_cast<unsigned long long>(goal.planning_attempt_id),
        static_cast<unsigned long long>(trajectory), static_cast<unsigned>(status), reason.c_str());
    result.requested_goal = goal.pose;
    const bool same_goal = contract_active_ &&
      goal.exploration_session_id == contract_goal_.exploration_session_id &&
      goal.goal_id == contract_goal_.goal_id &&
      goal.planning_attempt_id == contract_goal_.planning_attempt_id;
    if (same_goal && contract_effective_goal_valid_) {
      result.effective_goal_valid = true;
      result.effective_goal = goal.pose;
      result.effective_goal.position.x = end_pt_.x();
      result.effective_goal.position.y = end_pt_.y();
      result.effective_goal.position.z = end_pt_.z();
      result.goal_adjusted = goalPositionAdjusted(goal.pose, result.effective_goal);
    }
    if (status == gbplanner3_interfaces::msg::PlanningResult::READY && same_goal) {
      const auto &info = planner_manager_->local_data_;
      result.trajectory_start_time = info.start_time_;
      const Eigen::Vector3d endpoint = info.position_traj_.evaluateDeBoorT(info.duration_);
      result.trajectory_end.x = endpoint.x();
      result.trajectory_end.y = endpoint.y();
      result.trajectory_end.z = endpoint.z();
      // One atomic publication binds the exact spline to the full goal identity.
      // Native Bspline arrival order or its process-local ID alone is insufficient.
      gbplanner3_interfaces::msg::ScanTrajectory envelope;
      envelope.result = result;
      envelope.trajectory = contract_native_trajectory_;
      contract_trajectory_pub_->publish(envelope);
    }
    contract_pub_->publish(result);
  }

  // Only planning intent is snapshotted. Live odometry and the map keep updating.
  // Preserve the highest allocated trajectory ID even when a candidate rolls back.
  std::function<void()> SCANReplanFSM::captureContractState()
  {
    const double adopted_height = ground_goal_height_;
    const auto goal = contract_goal_;
    const auto native = contract_native_trajectory_;
    const auto local = planner_manager_->local_data_;
    const auto global = planner_manager_->global_data_;
    const auto state = exec_state_;
    const auto init = init_pt_, start = start_pt_, velocity = start_vel_, acceleration = start_acc_;
    const auto end = end_pt_, end_velocity = end_vel_, target = local_target_pt_, target_velocity = local_target_vel_;
    const bool active = contract_active_, effective = contract_effective_goal_valid_;
    const bool trigger = trigger_, have_target = have_target_, new_target = have_new_target_;
    const bool escape = flag_escape_emergency_;
    const int failures = contract_failures_, replan_failures = replan_fail_count_;
    return [this, adopted_height, goal, native, local, global, state, init, start, velocity, acceleration,
            end, end_velocity, target, target_velocity, active, effective, trigger,
            have_target, new_target, escape, failures, replan_failures]() {
      const auto allocated = planner_manager_->local_data_.traj_id_;
      ground_goal_height_ = adopted_height;
      contract_goal_ = goal;
      contract_native_trajectory_ = native;
      planner_manager_->local_data_ = local;
      planner_manager_->local_data_.traj_id_ = std::max(allocated, local.traj_id_);
      planner_manager_->global_data_ = global;
      exec_state_ = state;
      init_pt_ = init; start_pt_ = start; start_vel_ = velocity; start_acc_ = acceleration;
      end_pt_ = end; end_vel_ = end_velocity;
      local_target_pt_ = target; local_target_vel_ = target_velocity;
      contract_active_ = active; contract_effective_goal_valid_ = effective;
      trigger_ = trigger; have_target_ = have_target; have_new_target_ = new_target;
      flag_escape_emergency_ = escape;
      contract_failures_ = failures; replan_fail_count_ = replan_failures;
    };
  }

  void SCANReplanFSM::discardHandoff(const std::string &reason)
  {
    if (!handoff_commit_) return;
    handoff_commit_ = {};
    contractResult(handoff_goal_, gbplanner3_interfaces::msg::PlanningResult::BLOCKED,
                   reason, handoff_trajectory_id_);
  }

  void SCANReplanFSM::retireContract()
  {
    discardHandoff("HANDOFF_ACTIVE_GOAL_RETIRED_STOP_REQUIRED");
    contract_active_ = false;
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    changeFSMExecState(WAIT_TARGET, "CONTRACT_RETIRED");
  }

  void SCANReplanFSM::contractExecutionCallback(
      gbplanner3_interfaces::msg::ExecutionObservation::ConstSharedPtr msg)
  {
    using Observation = gbplanner3_interfaces::msg::ExecutionObservation;
    if (handoff_commit_ && msg->exploration_session_id == handoff_goal_.exploration_session_id &&
        msg->goal_id == handoff_goal_.goal_id &&
        msg->planning_attempt_id == handoff_goal_.planning_attempt_id &&
        (msg->trajectory_id == 0 || msg->trajectory_id == handoff_trajectory_id_)) {
      if (msg->status == Observation::RUNNING) {
        if (msg->reason != "CONTROLLER_ACCEPTED" || msg->trajectory_id == 0 ||
            msg->trajectory_id != handoff_trajectory_id_) return;
        auto commit = std::move(handoff_commit_);
        handoff_commit_ = {};
        commit();
        RCLCPP_INFO(node_->get_logger(), "SCAN_HANDOFF_COMMITTED goal=%llu trajectory=%llu",
          static_cast<unsigned long long>(contract_goal_.goal_id),
          static_cast<unsigned long long>(handoff_trajectory_id_));
      } else {
        handoff_commit_ = {};
        if (msg->reason == "HANDOFF_REJECTED_KEEP_PREVIOUS") {
          RCLCPP_WARN(node_->get_logger(), "SCAN_HANDOFF_REJECTED_KEEP_PREVIOUS reason=%s", msg->reason.c_str());
        } else {
          // Once READY was published the controller may already have switched.
          // Only an explicit rejected START proves that restoring old intent is valid.
          retireContract();
        }
      }
      return;
    }
    if (!contract_active_ || msg->status == gbplanner3_interfaces::msg::ExecutionObservation::RUNNING ||
        msg->exploration_session_id != contract_goal_.exploration_session_id ||
        msg->goal_id != contract_goal_.goal_id ||
        msg->planning_attempt_id != contract_goal_.planning_attempt_id ||
        (msg->trajectory_id != 0 && msg->trajectory_id != static_cast<uint64_t>(contract_native_trajectory_.traj_id))) return;
    if (handoff_commit_ && msg->status == Observation::SUCCESS &&
        msg->reason == "MEASURED_GOAL_POSE_STOP_CONFIRMED") {
      // The predecessor can finish while the successor START ack is in flight.
      // Keep its completion final, but retain the prepared successor transaction.
      contract_active_ = false;
      have_target_ = have_new_target_ = trigger_ = false;
      changeFSMExecState(WAIT_TARGET, "HANDOFF_PREDECESSOR_FINISHED");
      return;
    }
    // Cancellation events retire intent; executor alone proves physical stop.
    retireContract();
  }

  void SCANReplanFSM::contractGoalCallback(gbplanner3_interfaces::msg::LocalGoal::ConstSharedPtr msg)
  {
    using Result = gbplanner3_interfaces::msg::PlanningResult;
    const auto key = std::make_tuple(msg->exploration_session_id, msg->goal_id, msg->planning_attempt_id);
    if (contract_seen_.count(key)) return;
    // Fail closed when this process' replay cache fills; never evict old identities.
    if (contract_seen_.size() >= 4096) {
      contractResult(*msg, Result::ABORTED, "IDENTITY_CACHE_FULL_RESTART_REQUIRED"); return;
    }
    contract_seen_.insert(key);
    const auto now = node_->now();
    const auto clock_type = node_->get_clock()->get_clock_type();
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!msg->exploration_session_id || !msg->goal_id || !msg->planning_attempt_id ||
        !msg->source_path_id || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(qnorm) || std::abs(qnorm-1.0)>0.01 ||
        !std::isfinite(msg->planning_radius) || msg->planning_radius<=0 ||
        msg->header.frame_id.empty() || msg->header.frame_id != contract_odom_header_.frame_id ||
        !have_odom_ ||
        (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_) ||
        !odom_pos_.allFinite() ||
        rclcpp::Time(msg->valid_until, clock_type) <= now) {
      contractResult(*msg, Result::ABORTED, "INVALID_OR_STALE_INPUT"); return;
    }
    const bool handoff = contract_active_ && allow_goal_handoff_ && !contract_planning_only_ &&
      !handoff_commit_ && msg->exploration_session_id == contract_goal_.exploration_session_id &&
      msg->goal_id > contract_goal_.goal_id && msg->planning_attempt_id > contract_goal_.planning_attempt_id;
    if (handoff_commit_ || (contract_active_ && !handoff)) {
      contractResult(*msg, Result::ABORTED, "BUSY_NO_CONTROLLER_CANCEL_ACK"); return;
    }
    const auto map_stamp = planner_manager_->grid_map_->lastIntegratedCloudStampNs();
    // Require an initialized map, without rejecting goals based on cloud age.
    // User decision (2026-09-22): disable map freshness admission checking.
    if (map_stamp <= 0) {
      contractResult(*msg, Result::ABORTED, "MAP_NOT_READY"); return;
    }
    const bool reference_mode = navi_mode_ == NAVI_MODE::REFERENCE_PATH;
    std::vector<Eigen::Vector3d> reference;
    if (reference_mode) {
      if (msg->reference_path.size() < 2 || planning_horizon_ > msg->planning_radius + 1e-6) {
        contractResult(*msg, Result::ABORTED, "INVALID_REFERENCE_OR_LOCAL_HORIZON"); return;
      }
      for (const auto &wp : msg->reference_path) {
        const Eigen::Vector3d point(wp.position.x, wp.position.y, wp.position.z);
        const auto &orientation = wp.orientation;
        const double norm = orientation.x*orientation.x + orientation.y*orientation.y +
                            orientation.z*orientation.z + orientation.w*orientation.w;
        if (!point.allFinite() || !std::isfinite(norm) || std::abs(norm-1.0) > 0.01) {
          contractResult(*msg, Result::ABORTED, "INVALID_REFERENCE_POSE"); return;
        }
        if (reference.empty() || (point-reference.back()).norm() > 1e-6)
          reference.push_back(point);
      }
      // All poses are already body positions in the contract frame. Do not
      // apply the legacy initial_path callback's body-height addition.
      if (reference.size() < 2 || (reference.front()-odom_pos_).norm() > 1.0 ||
          (reference.back()-Eigen::Vector3d(p.x,p.y,p.z)).norm() > 1e-6) {
        contractResult(*msg, Result::ABORTED, "REFERENCE_ENDPOINT_OR_START_MISMATCH"); return;
      }
    } else if (!msg->reference_path.empty()) {
      contractResult(*msg, Result::ABORTED, "REFERENCE_REQUIRES_MODE_3"); return;
    }
    if (handoff) {
      auto restore_previous = captureContractState();
      contract_goal_ = *msg;
    ground_goal_height_ = msg->pose.position.z;
      contract_active_ = true;
      contract_effective_goal_valid_ = true;
      contract_failures_ = 0;
      trigger_ = true;
      init_pt_ = odom_pos_;
      end_pt_ = Eigen::Vector3d(p.x, p.y, p.z);
      end_vel_.setZero();
      setStartStateFromOdomOrCurrentTraj();
      bool success = reference_mode ? planGlobalTrajByWaypoints(reference) :
        planner_manager_->planGlobalTraj(start_pt_, start_vel_, start_acc_, end_pt_,
                                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      if (success && !reference_mode) success = adjustGlobalTargetIfOccupied();
      if (success) {
        have_target_ = true;
        have_new_target_ = true;
        for (int attempt = 0; attempt < 3; ++attempt) {
          success = callReboundReplan(true, attempt != 0);
          if (success) break;
        }
      }
      if (success) {
        exec_state_ = EXEC_TRAJ;
        replan_fail_count_ = 0;
        flag_escape_emergency_ = true;
        handoff_goal_ = *msg;
        handoff_trajectory_id_ = contract_native_trajectory_.traj_id;
        handoff_commit_ = captureContractState();
      } else {
        contractResult(*msg, Result::NO_PATH, "HANDOFF_PREPARE_FAILED_KEEP_PREVIOUS");
      }
      restore_previous();
      RCLCPP_INFO(node_->get_logger(), "SCAN_HANDOFF_PREPARED goal=%llu ready=%d previous_goal=%llu",
        static_cast<unsigned long long>(msg->goal_id), success,
        static_cast<unsigned long long>(contract_goal_.goal_id));
      return;
    }
    contract_goal_ = *msg;
    ground_goal_height_ = msg->pose.position.z;
    contract_effective_goal_valid_ = false;
    contract_active_ = true;
    contract_failures_ = 0;
    if (reference_mode) {
      trigger_ = true;
      init_pt_ = odom_pos_;
      contract_effective_goal_valid_ = true;
      if (!planGlobalTrajByWaypoints(reference)) {
        contractResult(*msg, Result::NO_PATH, "SCAN_REFERENCE_PLAN_FAILED");
        retireContract();
        return;
      }
      changeFSMExecState(GEN_NEW_TRAJ, "CONTRACT_REFERENCE");
      RCLCPP_INFO(node_->get_logger(), "Contract reference accepted: %zu points, local horizon %.2f m",
                  reference.size(), planning_horizon_);
      return;
    }
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;
    path->poses.push_back(pose);
    waypointCallback(path);
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    const auto &q = msg->pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (msg->header.frame_id != contract_odom_header_.frame_id ||
        !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 ||
        !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal with invalid frame/pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (!contract_active_ && msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y,
        contract_active_ ? msg->poses[0].pose.position.z : rviz_goal_height_;
    if (contract_active_) contract_effective_goal_valid_ = true;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      if (!contract_mode_ && navi_mode_ == NAVI_MODE::MANUAL_TARGET) {
        manual_view_goal_ = msg->poses[0];
        manual_view_goal_.header = msg->header;
        manual_view_goal_valid_ = true;
      }
      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
      if (contract_active_) {
        contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::NO_PATH, "SCAN_GLOBAL_PLAN_FAILED");
        contract_active_ = false;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "CONTRACT");
      }
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  bool SCANReplanFSM::isExplorationMode() const
  {
    return navi_mode_ == NAVI_MODE::EXPLORATION;
  }

  bool SCANReplanFSM::explorationWaypointIsFresh() const
  {
    return exploration_waypoint_received_ &&
           isExplorationWaypointFresh(
               node_->now().seconds(), exploration_waypoint_time_.seconds(),
               exploration_waypoint_timeout_);
  }

  bool SCANReplanFSM::explorationReferencePathIsFresh() const
  {
    return exploration_reference_path_received_ &&
           isExplorationWaypointFresh(
               node_->now().seconds(), exploration_reference_path_time_.seconds(),
               exploration_waypoint_timeout_);
  }

  bool SCANReplanFSM::hasActiveLocalTrajectory() const
  {
    const LocalTrajData &info = planner_manager_->local_data_;
    if (info.start_time_.seconds() < 1e-5 || info.duration_ <= 1e-5)
      return false;
    const double elapsed = (node_->now() - info.start_time_).seconds();
    return elapsed >= 0.0 && elapsed < info.duration_ - 0.05;
  }

  void SCANReplanFSM::explorationWaypointCallback(
      const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg)
  {
    if (!msg || !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
        !std::isfinite(msg->point.z))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore invalid exploration waypoint");
      return;
    }
    if (!msg->header.frame_id.empty() && !self_inflation_frame_id_.empty() &&
        msg->header.frame_id != self_inflation_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Ignore exploration waypoint in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), self_inflation_frame_id_.c_str());
      return;
    }

    Eigen::Vector3d waypoint(msg->point.x, msg->point.y, msg->point.z);
    exploration_waypoint_time_ = node_->now();
    exploration_waypoint_received_ = true;
    if (have_odom_ && !explorationReferencePathIsFresh())
    {
      Eigen::Vector3d candidate_direction = waypoint - odom_pos_;
      candidate_direction.z() = 0.0;
      constexpr int kDirectionConfirmations = 2;
      const Eigen::Vector3d committed_direction =
          have_target_ ? exploration_direction_ : Eigen::Vector3d::Zero();
      const ExplorationDirectionDecision decision = considerExplorationDirection(
          committed_direction, candidate_direction,
          exploration_direction_replan_threshold_, kDirectionConfirmations,
          exploration_direction_gate_);
      if (!decision.accept)
      {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Holding exploration direction change (%d/%d confirmations)",
            exploration_direction_gate_.pending_count, kDirectionConfirmations);
        return;
      }
      if (decision.request_replan)
        exploration_early_replan_requested_ = true;
    }

    exploration_waypoint_ = waypoint;
    if (!exploration_home_reached_ || !exploration_finished_)
      trigger_ = true;
  }

  void SCANReplanFSM::explorationReferencePathCallback(
      const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.size() < 2)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Ignore exploration reference path with fewer than two poses");
      return;
    }
    if (!msg->header.frame_id.empty() && !self_inflation_frame_id_.empty() &&
        msg->header.frame_id != self_inflation_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Ignore exploration reference path in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), self_inflation_frame_id_.c_str());
      return;
    }

    std::vector<Eigen::Vector3d> ordered_path;
    ordered_path.reserve(msg->poses.size());
    for (const auto &pose : msg->poses)
    {
      ordered_path.emplace_back(
          pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    const Eigen::Vector3d anchor = have_odom_ ? odom_pos_ : ordered_path.front();
    const ExplorationReferencePrefix prefix = buildOrderedExplorationPrefix(
        anchor, ordered_path, planning_horizon_);
    if (!prefix.valid)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Ignore invalid ordered exploration reference path");
      return;
    }

    if (have_odom_)
    {
      constexpr int kDirectionConfirmations = 2;
      const Eigen::Vector3d committed_direction =
          have_target_ ? exploration_direction_ : Eigen::Vector3d::Zero();
      const ExplorationDirectionDecision decision = considerExplorationDirection(
          committed_direction, prefix.initial_direction,
          exploration_direction_replan_threshold_, kDirectionConfirmations,
          exploration_reference_direction_gate_);
      if (!decision.accept)
      {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Holding ordered exploration path direction change (%d/%d confirmations)",
            exploration_reference_direction_gate_.pending_count,
            kDirectionConfirmations);
        return;
      }
      if (decision.request_replan)
        exploration_early_replan_requested_ = true;
    }

    exploration_reference_path_ = std::move(ordered_path);
    exploration_reference_path_time_ = node_->now();
    exploration_reference_path_received_ = true;
    exploration_reference_path_pending_ = true;
    exploration_direction_ = prefix.initial_direction;
    if (!exploration_home_reached_ || !exploration_finished_)
      trigger_ = true;
  }

  void SCANReplanFSM::explorationFinishCallback(
      const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    const bool changed = exploration_finished_ != msg->data;
    exploration_finished_ = msg->data;
    if (changed)
    {
      exploration_direction_gate_ = ExplorationDirectionGate{};
      exploration_reference_direction_gate_ = ExplorationDirectionGate{};
      exploration_reference_path_received_ = false;
      exploration_reference_path_pending_ = false;
      exploration_reference_path_.clear();
    }
    if (!exploration_finished_)
      exploration_home_reached_ = false;
    if (changed && exploration_finished_ && have_target_)
      exploration_early_replan_requested_ = true;
  }

  bool SCANReplanFSM::planExplorationGlobalTrajectory()
  {
    if (!exploration_home_ready_ || !explorationWaypointIsFresh())
      return false;

    Eigen::Vector3d map_origin;
    Eigen::Vector3d map_size;
    planner_manager_->grid_map_->getRegion(map_origin, map_size);
    const ExplorationTarget target = computeExplorationTarget(
        start_pt_, exploration_waypoint_, home_position_, exploration_finished_,
        planning_horizon_, map_origin, map_size,
        planner_manager_->grid_map_->getResolution(),
        exploration_return_stop_distance_);

    bool used_ordered_reference = false;
    bool global_plan_succeeded = false;
    ExplorationReferencePrefix reference_prefix;
    if (!target.fixed_home && exploration_reference_path_pending_ &&
        explorationReferencePathIsFresh())
    {
      reference_prefix = buildOrderedExplorationPrefix(
          start_pt_, exploration_reference_path_, planning_horizon_);
      if (reference_prefix.valid)
      {
        end_pt_ = reference_prefix.waypoints.back();
        ExplorationTarget terminal_target;
        terminal_target.valid = true;
        terminal_target.direction = reference_prefix.terminal_direction;
        end_vel_ = explorationTerminalVelocity(
            terminal_target, planner_manager_->pp_.max_vel_);
        global_plan_succeeded = planner_manager_->planGlobalTrajWaypoints(
            start_pt_, start_vel_, start_acc_, reference_prefix.waypoints,
            end_vel_, Eigen::Vector3d::Zero());
        used_ordered_reference = global_plan_succeeded;
        if (!global_plan_succeeded)
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Unable to generate trajectory from ordered exploration path; using waypoint fallback");
        }
      }
    }

    if (!global_plan_succeeded && !target.valid)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "No valid rolling exploration target inside the current map");
      return false;
    }

    if (!global_plan_succeeded &&
        !allowExplorationWaypointFallback(
            target.fixed_home, exploration_reference_path_received_))
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Ordered exploration reference is unavailable; refusing waypoint fallback");
      return false;
    }

    if (!global_plan_succeeded)
    {
      end_pt_ = target.position;
      end_vel_ = explorationTerminalVelocity(
          target, planner_manager_->pp_.max_vel_);
      global_plan_succeeded = planner_manager_->planGlobalTraj(
          start_pt_, start_vel_, start_acc_, end_pt_, end_vel_,
          Eigen::Vector3d::Zero());
      if (!global_plan_succeeded)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Unable to generate rolling exploration trajectory");
        return false;
      }
    }
    if (!adjustGlobalTargetIfOccupied())
      return false;

    exploration_reference_path_pending_ = false;
    exploration_direction_ = used_ordered_reference
                                 ? reference_prefix.initial_direction
                                 : target.direction;
    exploration_fixed_home_ = !used_ordered_reference && target.fixed_home;
    have_target_ = true;
    have_new_target_ = true;
    init_pt_ = start_pt_;

    constexpr double sample_dt = 0.1;
    const double duration = planner_manager_->global_data_.global_duration_;
    std::vector<Eigen::Vector3d> global_traj;
    global_traj.reserve(static_cast<size_t>(std::ceil(duration / sample_dt)) + 1);
    for (double t = 0.0; t < duration; t += sample_dt)
      global_traj.push_back(planner_manager_->global_data_.global_traj_.evaluate(t));
    global_traj.push_back(planner_manager_->global_data_.global_traj_.evaluate(duration));
    visualization_->displayGlobalPathList(global_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Exploration target [%.2f, %.2f, %.2f], source=%s, fixed_home=%s, terminal_speed=%.2f",
        end_pt_(0), end_pt_(1), end_pt_(2),
        used_ordered_reference ? "ordered_reference" : "waypoint_fallback",
        exploration_fixed_home_ ? "true" : "false", end_vel_.norm());
    return true;
  }

  void SCANReplanFSM::stopExpiredExplorationIntent()
  {
    RCLCPP_WARN(node_->get_logger(), "Exploration waypoint timed out; stopping current trajectory");
    need_hover_stop_ = true;
    flag_escape_emergency_ = true;
    have_target_ = false;
    trigger_ = false;
    exploration_early_replan_requested_ = false;
    exploration_preserve_on_replan_failure_ = false;
    exploration_direction_gate_ = ExplorationDirectionGate{};
    exploration_reference_direction_gate_ = ExplorationDirectionGate{};
    exploration_reference_path_received_ = false;
    exploration_reference_path_pending_ = false;
    exploration_reference_path_.clear();
    changeFSMExecState(EMERGENCY_STOP, "EXPLORATION_TIMEOUT");
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    if (ground_height_follow_ && msg->header.frame_id == self_inflation_frame_id_) {
      ground_height_.add(rclcpp::Time(msg->header.stamp).seconds(),
                        msg->pose.pose.position.z, node_->now().seconds());
    }
    contract_odom_header_ = msg->header;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (isExplorationMode() && !exploration_home_ready_ && odom_pos_.allFinite())
    {
      home_position_ = odom_pos_;
      exploration_home_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Recorded exploration home at [%.2f, %.2f, %.2f]",
                  home_position_(0), home_position_(1), home_position_(2));
    }

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    if (fsm_state_pub_ && pre_s != int(new_state))
    {
      std_msgs::msg::String message;
      message.data = state_str[int(new_state)];
      fsm_state_pub_->publish(message);
    }
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    scan_diagnostics::CallbackTiming timing(node_, "fsm");
    // The old controller keeps moving while the candidate START is acknowledged.
    // Collision checking remains active against the still executing old trajectory.
    if (handoff_commit_) {
      const auto now = node_->now();
      const auto clock = node_->get_clock()->get_clock_type();
      if (now >= rclcpp::Time(handoff_goal_.valid_until, clock) ||
          (contract_active_ && now >= rclcpp::Time(contract_goal_.valid_until, clock))) {
        discardHandoff("HANDOFF_LEASE_EXPIRED_STOP_REQUIRED");
        retireContract();
      }
      return;
    }
    if (contract_active_ && node_->now() >= rclcpp::Time(contract_goal_.valid_until, node_->get_clock()->get_clock_type())) {
      contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::TIMEOUT, "PLANNING_LEASE_EXPIRED");
      contract_active_ = false;
      have_target_ = false;
      changeFSMExecState(WAIT_TARGET, "CONTRACT");
    }

    updateLocalTrajTimeFreeze();

    if (isExplorationMode() && have_target_ && !exploration_fixed_home_ &&
        exec_state_ != INIT && exec_state_ != WAIT_TARGET &&
        exec_state_ != EMERGENCY_STOP && !explorationWaypointIsFresh())
    {
      stopExpiredExplorationIntent();
      return;
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (isExplorationMode() && !have_target_)
      {
        if (exploration_home_reached_ || !explorationWaypointIsFresh())
          return;
        setStartStateFromOdomOrCurrentTraj();
        if (!planExplorationGlobalTrajectory())
          return;
        changeFSMExecState(GEN_NEW_TRAJ, "EXPLORATION");
        break;
      }
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (contract_mode_ && contract_active_) {
        if (node_->now() >= rclcpp::Time(contract_goal_.valid_until, node_->get_clock()->get_clock_type())) {
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::TIMEOUT, "PLANNING_LEASE_EXPIRED_DURING_PLAN");
        } else if (success && !contract_planning_only_) {
          contract_failures_ = 0;
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "CONTRACT_LIVE");
          flag_escape_emergency_ = true;
          break;
        } else if (success) {
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::READY,
            "PLANNED_NOT_EXECUTED;terminal_yaw_handled_by_executor;native_traj_id=" + std::to_string(planner_manager_->local_data_.traj_id_),
            static_cast<uint64_t>(planner_manager_->local_data_.traj_id_));
        } else if (++contract_failures_ < 3) {
          break;
        } else {
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::NO_PATH, "SCAN_REBOUND_FAILED_3_TRIES");
        }
        contract_active_ = false;
        have_target_ = false;
        have_new_target_ = false;
        changeFSMExecState(WAIT_TARGET, "CONTRACT_PLANNING_ONLY");
        break;
      }
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        exploration_preserve_on_replan_failure_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else if (isExplorationMode() && exploration_preserve_on_replan_failure_ &&
               explorationWaypointIsFresh() && hasActiveLocalTrajectory())
      {
        replan_fail_count_++;
        exploration_preserve_on_replan_failure_ = false;
        RCLCPP_WARN(node_->get_logger(),
                    "Early exploration replan failed; keeping the active trajectory");
        changeFSMExecState(EXEC_TRAJ, "EXPLORATION_KEEP");
      }
      else
      {
        if (contract_mode_ && contract_active_) {
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::BLOCKED,
              "SCAN_LIVE_REPLAN_FAILED");
          retireContract();
          break;
        }
        exploration_preserve_on_replan_failure_ = false;
        replan_fail_count_++;
        visualization_->clearOptimalTraj(0);
        callEmergencyStop(odom_pos_);
        changeFSMExecState(GEN_NEW_TRAJ, "REPLAN_STOP");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // A controller turn/terminal-yaw hold freezes execution progress. Keep
      // the native collision timer active, but do not periodically replace
      // the very trajectory whose endpoint orientation is being completed.
      if (contract_mode_ && contract_active_ && go2_execution_frozen_) {
        const Eigen::Vector3d local_end = info->position_traj_.evaluateDeBoorT(info->duration_);
        if ((odom_pos_-local_end).norm() <= reference_segment_reached_tolerance_ &&
            (local_end-end_pt_).norm() > reference_segment_reached_tolerance_) {
          changeFSMExecState(REPLAN_TRAJ, "CONTRACT_LOCAL_ARRIVAL");
        }
        return;
      }

      if (isExplorationMode() && shouldStartExplorationReplan(
              exploration_early_replan_requested_, go2_execution_frozen_))
      {
        exploration_early_replan_requested_ = false;
        exploration_preserve_on_replan_failure_ = true;
        changeFSMExecState(REPLAN_TRAJ, "EXPLORATION_DIRECTION");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (contract_mode_ && contract_active_ && navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
            (info->position_traj_.evaluateDeBoorT(info->duration_) - end_pt_).norm() > no_replan_thresh_)
        {
          // A local spline ending is not completion of the full reference.
          // Preserve the ordered global curve and advance its local target.
          changeFSMExecState(REPLAN_TRAJ, "CONTRACT_REFERENCE_CONTINUE");
          return;
        }
        if (isExplorationMode())
        {
          if (exploration_fixed_home_)
          {
            exploration_home_reached_ = true;
            have_target_ = false;
            trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EXPLORATION_HOME");
            return;
          }

          if (explorationWaypointIsFresh())
          {
            setStartStateFromOdomOrCurrentTraj();
            if (planExplorationGlobalTrajectory())
            {
              changeFSMExecState(GEN_NEW_TRAJ, "EXPLORATION_ROLL");
              return;
            }
          }

          RCLCPP_WARN(node_->get_logger(),
                      "Exploration trajectory ended without a valid continuation; stopping");
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(EMERGENCY_STOP, "EXPLORATION_END");
          return;
        }

        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (isExplorationMode() && exploration_direction_.head<2>().norm() > 1e-3)
      to_goal = exploration_direction_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (isExplorationMode())
    {
      const bool keep_active_ordered_reference =
          explorationReferencePathIsFresh() &&
          !exploration_reference_path_pending_ &&
          planner_manager_->global_data_.global_duration_ > 1e-3;
      if (!keep_active_ordered_reference && !planExplorationGlobalTrajectory())
        return false;
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // The reference-path mode owns a multi-waypoint global trajectory. Do
      // not replace it with a shortcut from the current pose to the final
      // goal; getLocalTarget() will advance last_progress_time_ on it.
      if (planner_manager_->global_data_.global_duration_ <= 1e-3)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Reference path is unavailable during local replan");
        return false;
      }
    }
    else if (!planner_manager_->planGlobalTraj(
                 start_pt_,
                 start_vel_,
                 start_acc_,
                 end_pt_,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!isExplorationMode() && !adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (isExplorationMode() && exploration_direction_.head<2>().norm() > 1e-3)
      to_goal = exploration_direction_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    scan_diagnostics::CallbackTiming timing(node_, "collision");
    if (contract_planning_only_) return; // No executing trajectory in offline mode.

    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (handoff_commit_) {
          // A START may already be accepted with its acknowledgement in flight.
          // A real collision is an explicit stop, never a silent handoff rollback.
          discardHandoff("HANDOFF_ACTIVE_COLLISION_STOP_REQUIRED");
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::BLOCKED,
                         "HANDOFF_ACTIVE_COLLISION_STOP_REQUIRED");
          retireContract();
          return;
        }
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (contract_mode_ && contract_active_) {
            contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::BLOCKED,
                "SCAN_LIVE_COLLISION_REPLAN_FAILED");
            retireContract();
            return;
          }
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    // Update the effective Z at an existing replan boundary, never by creating
    // a new contract goal. Keep the original requested XYZ/IDs for correlation.
    double height;
    if (ground_height_follow_ && contract_active_ && navi_mode_ == NAVI_MODE::MANUAL_TARGET &&
        ground_height_.updateNeeded(node_->now().seconds(), ground_goal_height_, height)) {
      const auto previous_end = end_pt_;
      const auto previous_global = planner_manager_->global_data_;
      end_pt_.z() = height;
      if (!planner_manager_->planGlobalTraj(start_pt_, start_vel_, start_acc_, end_pt_,
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) || !adjustGlobalTargetIfOccupied()) {
        end_pt_ = previous_end;
        planner_manager_->global_data_ = previous_global;
        return false;
      }
      ground_goal_height_ = height;
      flag_use_poly_init = true;
      RCLCPP_INFO(node_->get_logger(), "GROUND_HEIGHT_UPDATE old=%.3f mean=%.3f effective=%.3f",
                  previous_end.z(), height, end_pt_.z());
    }
    const double reference_target_time = getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj, reference_target_time);
    if (!plan_success) {
      RCLCPP_WARN(node_->get_logger(),
          "SCAN_REPLAN_CONTEXT session=%llu goal=%llu attempt=%llu state=%d poly_init=%d random=%d plan_distance=%.6f odom_local_distance=%.6f odom_goal_distance=%.6f no_replan_threshold=%.6f start=[%.6f,%.6f,%.6f] odom=[%.6f,%.6f,%.6f] local_target=[%.6f,%.6f,%.6f] start_speed=%.6f start_acc=%.6f",
          static_cast<unsigned long long>(contract_goal_.exploration_session_id),
          static_cast<unsigned long long>(contract_goal_.goal_id),
          static_cast<unsigned long long>(contract_goal_.planning_attempt_id),
          static_cast<int>(exec_state_), static_cast<int>(have_new_target_ || flag_use_poly_init),
          static_cast<int>(flag_randomPolyTraj), (start_pt_-local_target_pt_).norm(),
          (odom_pos_-local_target_pt_).norm(), (odom_pos_-end_pt_).norm(), no_replan_thresh_,
          start_pt_.x(), start_pt_.y(), start_pt_.z(), odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
          local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(), start_vel_.norm(), start_acc_.norm());
    }
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      if (contract_active_ && !contract_planning_only_ &&
          node_->now() >= rclcpp::Time(contract_goal_.valid_until, node_->get_clock()->get_clock_type()))
        return false; // Never publish a spline whose lease expired during optimization.
      if (contract_active_) {
        contract_native_trajectory_ = bspline;
        if (!contract_planning_only_) {
          contractResult(contract_goal_, gbplanner3_interfaces::msg::PlanningResult::READY,
              "LIVE_NATIVE_TRAJECTORY", static_cast<uint64_t>(info->traj_id_));
        }
      }
      bspline_pub_->publish(bspline);
      publishGoalTrajectory(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  void SCANReplanFSM::publishGoalTrajectory(
      const scan_planner_msgs::msg::Bspline &trajectory, bool emergency)
  {
    if (contract_mode_) return;
    gbplanner3_interfaces::msg::ScanTrajectory out;
    out.trajectory = trajectory;
    out.result.header = contract_odom_header_;
    out.result.header.stamp = node_->now();
    out.result.trajectory_id = trajectory.traj_id;
    out.result.trajectory_start_time = trajectory.start_time;
    out.result.status = emergency ? gbplanner3_interfaces::msg::PlanningResult::ABORTED
                                  : gbplanner3_interfaces::msg::PlanningResult::READY;
    out.result.reason = emergency ? "NATIVE_EMERGENCY_STOP" : "MANUAL_GOAL_TRAJECTORY";
    if (!emergency && manual_view_goal_valid_ && navi_mode_ == NAVI_MODE::MANUAL_TARGET) {
      out.result.requested_goal = manual_view_goal_.pose;
      out.result.effective_goal = manual_view_goal_.pose;
      out.result.effective_goal.position.x = end_pt_.x();
      out.result.effective_goal.position.y = end_pt_.y();
      out.result.effective_goal.position.z = end_pt_.z();
      out.result.effective_goal_valid = true;
      out.result.goal_adjusted = goalPositionAdjusted(out.result.requested_goal, out.result.effective_goal);
    }
    const auto &info = planner_manager_->local_data_;
    const auto endpoint = info.position_traj_.evaluateDeBoorT(info.duration_);
    out.result.trajectory_end.x = endpoint.x();
    out.result.trajectory_end.y = endpoint.y();
    out.result.trajectory_end.z = endpoint.z();
    goal_trajectory_pub_->publish(out);
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);
    publishGoalTrajectory(bspline, true);

    return true;
  }

  void SCANReplanFSM::displayRemainingGlobalPath()
  {
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    const double start_t = std::max(
        0.0, std::min(global_data.last_progress_time_, duration));

    std::vector<Eigen::Vector3d> remaining_path;
    if (duration > 1e-3)
    {
      constexpr double sample_dt = 0.1;
      remaining_path.reserve(
          static_cast<size_t>(std::ceil((duration - start_t) / sample_dt)) + 1);
      for (double t = start_t; t < duration; t += sample_dt)
        remaining_path.push_back(global_data.getPosition(t));
      remaining_path.push_back(global_data.getPosition(duration));
    }

    visualization_->displayGlobalPathList(remaining_path, 0.1, 0);
  }

  double SCANReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t >= planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
      planner_manager_->global_data_.last_progress_time_ = std::max(
          planner_manager_->global_data_.last_progress_time_, dist_min_t);
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    auto admissibleAdjustment = [&](const Eigen::Vector3d &pt) {
      // A free start point is not a usable replacement for a blocked target.
      // Reference-mode adjustment must also stay inside the local horizon.
      return pt.allFinite() &&
          (navi_mode_ != NAVI_MODE::REFERENCE_PATH ||
           referenceTargetInRange((pt-start_pt_).norm(), planning_horizon_, 0.1)) &&
          targetOccupancy(pt) == 0;
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (admissibleAdjustment(pt))
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (admissibleAdjustment(pt))
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if (!isExplorationMode() || exploration_fixed_home_)
    {
      if ((end_pt_ - local_target_pt_).norm() <
          (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) /
              (2 * planner_manager_->pp_.max_acc_))
      {
        local_target_vel_ = Eigen::Vector3d::Zero();
      }
      else
      {
        local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      }
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
    }

    displayRemainingGlobalPath();
    return target_t;  // Preserve the selected branch/time after collision adjustment.
  }

} // namespace scan_planner
