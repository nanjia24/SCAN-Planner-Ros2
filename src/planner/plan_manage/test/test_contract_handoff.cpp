#include <gtest/gtest.h>
#include <plan_manage/scan_replan_fsm.h>

namespace scan_planner {
// Tests the real snapshot/ack callback without map mocks or issuing robot commands.
class SCANHandoffTestPeer {
public:
  static void run(bool accepted, bool predecessor_finishes = false) {
    auto node = std::make_shared<rclcpp::Node>("scan_handoff_snapshot_test");
    SCANReplanFSM fsm;
    fsm.node_ = node.get();
    fsm.planner_manager_.reset(new SCANPlannerManager);
    fsm.contract_active_ = fsm.contract_effective_goal_valid_ = true;
    fsm.trigger_ = fsm.have_target_ = true;
    fsm.have_new_target_ = false;
    fsm.flag_escape_emergency_ = true;
    fsm.exec_state_ = SCANReplanFSM::EXEC_TRAJ;
    fsm.init_pt_ = fsm.start_pt_ = fsm.start_vel_ = fsm.start_acc_ = Eigen::Vector3d::Zero();
    fsm.end_pt_ = Eigen::Vector3d(1, 0, 0);
    fsm.end_vel_ = fsm.local_target_pt_ = fsm.local_target_vel_ = Eigen::Vector3d::Zero();
    fsm.contract_goal_.exploration_session_id = 7;
    fsm.contract_goal_.goal_id = 1;
    fsm.contract_goal_.planning_attempt_id = 1;
    fsm.contract_native_trajectory_.traj_id = 10;
    auto &local = fsm.planner_manager_->local_data_;
    local.traj_id_ = 10;
    local.duration_ = 1;
    local.global_time_offset = 0;
    local.start_pos_.setZero();
    local.position_traj_ = UniformBspline(Eigen::MatrixXd::Zero(3, 6), 3, 0.2);
    local.velocity_traj_ = local.position_traj_.getDerivative();
    local.acceleration_traj_ = local.velocity_traj_.getDerivative();
    const auto zero = Eigen::Vector3d::Zero().eval();
    fsm.planner_manager_->global_data_.setGlobalTraj(
      PolynomialTraj::one_segment_traj_gen(zero, zero, zero, fsm.end_pt_, zero, zero, 1.0),
      rclcpp::Time(0));
    fsm.ground_goal_height_ = .25;
    auto rollback = fsm.captureContractState();
    fsm.ground_goal_height_ = .5;
    fsm.contract_goal_.goal_id = 2;
    fsm.contract_goal_.planning_attempt_id = 2;
    fsm.contract_native_trajectory_.traj_id = local.traj_id_ = 11;
    fsm.end_pt_.x() = 3;
    fsm.handoff_goal_ = fsm.contract_goal_;
    fsm.handoff_trajectory_id_ = 11;
    fsm.handoff_commit_ = fsm.captureContractState();
    rollback();
    EXPECT_EQ(fsm.contract_goal_.goal_id, 1u);
    EXPECT_DOUBLE_EQ(fsm.end_pt_.x(), 1);
    EXPECT_EQ(local.traj_id_, 11);  // Never reuse a published candidate ID.
    using Observation = gbplanner3_interfaces::msg::ExecutionObservation;
    if (predecessor_finishes) {
      auto done = std::make_shared<Observation>();
      done->exploration_session_id = 7;
      done->goal_id = done->planning_attempt_id = 1;
      done->trajectory_id = 10;
      done->status = Observation::SUCCESS;
      done->reason = "MEASURED_GOAL_POSE_STOP_CONFIRMED";
      fsm.contractExecutionCallback(done);
      EXPECT_TRUE(fsm.handoff_commit_);
      EXPECT_FALSE(fsm.contract_active_);
      EXPECT_EQ(fsm.exec_state_, SCANReplanFSM::WAIT_TARGET);
    }
    auto ack = std::make_shared<Observation>();
    ack->exploration_session_id = 7;
    ack->goal_id = ack->planning_attempt_id = 2;
    ack->trajectory_id = 11;
    ack->status = accepted ? Observation::RUNNING : Observation::ABORTED;
    ack->reason = accepted ? "CONTROLLER_ACCEPTED" : "HANDOFF_REJECTED_KEEP_PREVIOUS";
    if (accepted) {
      ack->reason = "TRAJECTORY_RECEIVED";
      fsm.contractExecutionCallback(ack);
      EXPECT_TRUE(fsm.handoff_commit_);
      ack->reason = "CONTROLLER_ACCEPTED";
      ack->trajectory_id = 0;
      fsm.contractExecutionCallback(ack);
      EXPECT_TRUE(fsm.handoff_commit_);
      ack->trajectory_id = 11;
    }
    fsm.contractExecutionCallback(ack);
    EXPECT_FALSE(fsm.handoff_commit_);
    EXPECT_EQ(fsm.contract_active_, accepted || !predecessor_finishes);
    EXPECT_EQ(fsm.contract_goal_.goal_id, accepted ? 2u : 1u);
    EXPECT_DOUBLE_EQ(fsm.end_pt_.x(), accepted ? 3 : 1);
    if (accepted) {
      ack->goal_id = ack->planning_attempt_id = 1;
      ack->trajectory_id = 10;
      ack->status = Observation::ABORTED;
      fsm.contractExecutionCallback(ack);
      EXPECT_TRUE(fsm.contract_active_);  // Late predecessor cannot retire successor.
      EXPECT_EQ(fsm.contract_goal_.goal_id, 2u);
    }
  }
};
TEST(ContractHandoff, AcceptedCommitsAndOldFeedbackIsIgnored) {
  rclcpp::init(0, nullptr);
  SCANHandoffTestPeer::run(true);
  rclcpp::shutdown();
}
TEST(ContractHandoff, ExplicitRejectedStartKeepsPrevious) {
  rclcpp::init(0, nullptr);
  SCANHandoffTestPeer::run(false);
  rclcpp::shutdown();
}
TEST(ContractHandoff, PredecessorCompletionDoesNotCancelPreparedSuccessor) {
  rclcpp::init(0, nullptr);
  SCANHandoffTestPeer::run(true, true);
  rclcpp::shutdown();
}
TEST(ContractHandoff, RejectedSuccessorDoesNotResumeCompletedPredecessor) {
  rclcpp::init(0, nullptr);
  SCANHandoffTestPeer::run(false, true);
  rclcpp::shutdown();
}
}  // namespace scan_planner

// Real manager/optimizer on synthetic known-free cells. No controller or PX4
// publisher; compare moving replans and end braking under the installed config.
TEST(ReferenceTimingIntegration, MovingReplansKeepCruiseAndEndBraking) {
  using namespace scan_planner;
  const char* config=std::getenv("SCAN_AUDIT_CONFIG"); ASSERT_NE(config,nullptr);
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("scan_planner_node",
      rclcpp::NodeOptions().arguments({"--ros-args","--params-file",config}).parameter_overrides({
        rclcpp::Parameter("grid_map.sliding_map_size_x",20.),
        rclcpp::Parameter("grid_map.sliding_map_size_y",4.),
        rclcpp::Parameter("grid_map.sliding_map_size_z",2.),
        rclcpp::Parameter("grid_map.ground_height",-1.)}));
  {
    SCANPlannerManager manager;
    auto vis=std::make_shared<PlanningVisualization>(node.get());
    manager.initPlanModules(node.get(),vis);
    Eigen::Vector3d origin,size;manager.grid_map_->getRegion(origin,size);
    const double res=manager.grid_map_->getResolution();
    for(double x=origin.x()+res/2;x<origin.x()+size.x();x+=res)
      for(double y=origin.y()+res/2;y<origin.y()+size.y();y+=res)
        for(double z=origin.z()+res/2;z<origin.z()+size.z();z+=res)
          manager.grid_map_->setOccupancy(Eigen::Vector3d(x,y,z),0);
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero(),goal(4,0,0);
    Eigen::Vector3d p(-4,0,0),v=zero,a=zero;
    const double nominal_v=node->get_parameter("manager.max_vel").as_double();
    const double nominal_a=node->get_parameter("manager.max_acc").as_double();
    double elapsed=0.;
    for(int cycle=0;cycle<12 && (p-goal).norm()>.1;++cycle) {
      const auto begin=std::chrono::steady_clock::now();
      ASSERT_TRUE(manager.planGlobalTraj(p,v,a,goal,zero,zero)) << "cycle=" << cycle << " speed=" << v.norm() << " acceleration=" << a.norm();
      double end=manager.global_data_.global_duration_;
      // Match real rolling-horizon selection without altering its geometry.
      for(double t=0;t<end;t+=.375) {
        if((manager.global_data_.getPosition(t)-p).norm()>=7.5){end=t;break;}
      }
      const auto target=manager.global_data_.getPosition(end);
      const Eigen::Vector3d target_v=(target-goal).norm()<.5 ? zero : manager.global_data_.getVelocity(end);
      ASSERT_TRUE(manager.reboundReplan(p,v,a,target,target_v,true,false,end)) << "cycle=" << cycle;
      auto& local=manager.local_data_;
      EXPECT_LT((local.position_traj_.evaluateDeBoorT(local.duration_)-target).norm(),.08);
      if((target-goal).norm()<.01) { EXPECT_LT(local.velocity_traj_.evaluateDeBoorT(local.duration_).norm(),.03); }
      double peak_speed=0.,peak_acc=0.;
      for(int sample=0;sample<=200;++sample) {
        const double t=local.duration_*sample/200.;
        peak_speed=std::max(peak_speed,local.velocity_traj_.evaluateDeBoorT(t).norm());
        peak_acc=std::max(peak_acc,local.acceleration_traj_.evaluateDeBoorT(t).norm());
      }
      EXPECT_LE(peak_speed,nominal_v+.05);
      EXPECT_LE(peak_acc,nominal_a+.05);
      const double dt=std::min(2.,local.duration_);
      p=local.position_traj_.evaluateDeBoorT(dt);
      v=local.velocity_traj_.evaluateDeBoorT(dt);
      a=local.acceleration_traj_.evaluateDeBoorT(dt);
      elapsed+=dt;
      std::cout << "MOVING_REPLAN_AUDIT cycle=" << cycle << " x=" << p.x() << " speed=" << v.norm()
                << " acc=" << a.norm() << " duration=" << local.duration_
                << " peak_speed=" << peak_speed << " peak_acc=" << peak_acc
                << " wall_ms=" << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count() << std::endl;
    }
    EXPECT_LT((p-goal).norm(),.1);
    EXPECT_LT(elapsed,12.5); // 8 m route, repeatedly replanned; old reference ~16+ s.

    // A local spline may be inside native feasibility tolerance but above the
    // nominal guide limit. Real executable boundary must survive guide fallback.
    const Eigen::Vector3d moving_start(-4,0,0), moving_v(nominal_v+.05,0,0), moving_a(.1,0,0);
    ASSERT_TRUE(manager.planGlobalTraj(moving_start,moving_v,moving_a,goal,zero,zero));
    const double guide_duration=manager.global_data_.global_duration_;
    ASSERT_TRUE(manager.reboundReplan(moving_start,moving_v,moving_a,goal,zero,true,false,guide_duration));
    EXPECT_LT((manager.local_data_.velocity_traj_.evaluateDeBoorT(0)-moving_v).norm(),1e-3);
    EXPECT_LT((manager.local_data_.acceleration_traj_.evaluateDeBoorT(0)-moving_a).norm(),1e-3);
    EXPECT_LT((manager.local_data_.position_traj_.evaluateDeBoorT(manager.local_data_.duration_)-goal).norm(),.08);
  }
  node.reset();rclcpp::shutdown();
}
