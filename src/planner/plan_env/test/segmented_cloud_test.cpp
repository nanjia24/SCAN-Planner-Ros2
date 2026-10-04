#include <plan_env/grid_map.h>
#include <stdexcept>
#include <iostream>

class GridMapSegmentedTest {
 public:
  static void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("grid_map.resolution", .1),
      rclcpp::Parameter("grid_map.sliding_map_size_x", 6.),
      rclcpp::Parameter("grid_map.sliding_map_size_y", 6.),
      rclcpp::Parameter("grid_map.sliding_map_size_z", 4.),
      rclcpp::Parameter("grid_map.ground_height", -2.),
      rclcpp::Parameter("grid_map.skip_pixel", 2),
      rclcpp::Parameter("grid_map.p_hit", .85), rclcpp::Parameter("grid_map.p_miss", .3),
      rclcpp::Parameter("grid_map.p_min", .12), rclcpp::Parameter("grid_map.p_max", .98),
      rclcpp::Parameter("grid_map.p_occ", .8), rclcpp::Parameter("grid_map.max_ray_length", 5.),
      rclcpp::Parameter("grid_map.double_cylinder_radius", .225),
      rclcpp::Parameter("grid_map.obstacles_inflation_z_up", 0.),
      rclcpp::Parameter("grid_map.obstacles_inflation_z_down", .2),
      rclcpp::Parameter("grid_map.need_extrinsic", false),
      rclcpp::Parameter("grid_map.map_sliding_en", false),
      rclcpp::Parameter("grid_map.use_segmented_cloud", true)});
    auto node = std::make_shared<rclcpp::Node>("segmented_cloud_test", options);
    GridMap map;
    map.initMap(node.get());
    const auto cloud_qos = map.cloud_sub_->get_actual_qos().get_rmw_qos_profile();
    const auto ground_qos = map.ground_cloud_sub_->get_actual_qos().get_rmw_qos_profile();
    require(cloud_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
            ground_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            "both segmented subscriptions must default to reliable delivery");
    require(cloud_qos.depth == 12 && ground_qos.depth == 12 && map.pair_queue_depth_ == 12,
            "segmented subscriptions and pair cache must share the bounded default depth");
    map.md_.ray_pos_ = Eigen::Vector3d(.05, .05, .05);
    map.md_.ray_q_ = Eigen::Quaterniond::Identity();
    map.md_.has_ray_pose_ = true;
    const Eigen::Vector3d ground(2.05, .05, .05), obstacle(1.05, .05, .05),
        intermediate(.55, .05, .05), beyond_ground(2.55, .05, .05);
    auto value = [&](const Eigen::Vector3d &point) {
      Eigen::Vector3i id; map.posToIndex(point, id);
      return map.md_.occupancy_buffer_[map.toAddress(id)];
    };
    auto batch = [&](std::vector<Eigen::Vector3d> points, std::vector<bool> hits) {
      map.md_.proj_points_ = points;
      map.md_.proj_points_cnt = points.size();
      map.segmented_batch_ = true;
      map.projected_hits_ = hits;
      map.raycastProcess();
    };
    const double unknown = value(ground);
    const double beyond_unknown = value(beyond_ground);
    batch({ground}, {false});
    require(value(ground) == unknown, "ground endpoint must remain unobserved, not hit or miss");
    require(value(beyond_ground) == beyond_unknown, "ground ray must not clear behind its endpoint");
    require(value(intermediate) == map.mp_.clamp_min_log_, "ground ray must clear intermediate cells");
    for (int i=0; i<4; ++i) batch({obstacle, ground, ground, ground}, {true,false,false,false});
    require(value(obstacle) >= map.mp_.min_occupancy_log_, "same batch ground clearing must not suppress obstacle hit");
    for (int i=0; i<12; ++i) batch({ground}, {false});
    require(value(obstacle) < map.mp_.min_occupancy_log_, "later clearing-only observations must remove old obstacle");

    // The raw path keeps the native hit/miss majority rule.
    const Eigen::Vector3d raw_cell(2.05, 1.05, .05);
    map.setCacheOccupancy(raw_cell, 1);
    map.setCacheOccupancy(raw_cell, 0);
    map.setCacheOccupancy(raw_cell, 0);
    map.md_.proj_points_ = {Eigen::Vector3d(.05, 2.05, .05)};
    map.md_.proj_points_cnt = 1;
    map.segmented_batch_ = false;
    map.projected_hits_.clear();
    map.raycastProcess();
    require(value(raw_cell) == map.mp_.clamp_min_log_, "raw majority miss must be preserved");

    // Pair mismatches must not produce an integrated observation.
    auto cloud = [](int sec, const std::string &frame) {
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      msg->header.stamp.sec=sec; msg->header.frame_id=frame; return msg;
    };
    map.segmentedCloudCallback(cloud(1,"lidar"), false);
    map.segmentedCloudCallback(cloud(2,"lidar"), true);
    require(map.cloud_pairs_.size()==2, "different timestamps must not pair");
    map.segmentedCloudCallback(cloud(1,"other"), true);
    require(map.cloud_pairs_.count(1'000'000'000LL)==0, "frame mismatch must be dropped");
    require(map.lastIntegratedCloudStampNs()==0, "mismatched clouds must never integrate");
    for(int i=3;i<15;++i) map.segmentedCloudCallback(cloud(i,"lidar"), true);
    require(map.cloud_pairs_.size()<=map.pair_queue_depth_, "pair cache must be bounded");
    map.sensor_poses_[10] = {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
    map.sensor_poses_[20] = {Eigen::Vector3d(2,0,0), Eigen::Quaterniond(Eigen::AngleAxisd(1.,Eigen::Vector3d::UnitZ()))};
    require(!map.setSegmentedPose(9) && !map.setSegmentedPose(21), "do not extrapolate missing odometry");
    require(map.setSegmentedPose(15), "bracketed odometry must interpolate");
    require((map.md_.ray_pos_-Eigen::Vector3d(1,0,0)).norm()<1e-9, "position must interpolate at cloud timestamp");
    require(std::abs(Eigen::AngleAxisd(map.md_.ray_q_).angle()-.5)<1e-9, "orientation must slerp");
    map.cloud_pairs_.clear();
    map.sensor_poses_.clear();
    map.sensor_poses_[1'000'000'000LL] = {Eigen::Vector3d(.05,.05,.05), Eigen::Quaterniond::Identity()};
    map.sensor_poses_[2'000'000'000LL] = {Eigen::Vector3d(.05,.05,.05), Eigen::Quaterniond::Identity()};
    const Eigen::Vector3d latest_pose(1.05,.05,.05);
    map.sensor_poses_[3'000'000'000LL] = {latest_pose, Eigen::Quaterniond::Identity()};
    map.mp_.map_sliding_en_ = true;
    map.md_.ray_pos_ = latest_pose;
    map.updateSlidingMap(latest_pose);
    const Eigen::Vector3i latest_origin = map.mp_.map_origin_idx_;
    auto pairCloud = [](float x) {
      pcl::PointCloud<pcl::PointXYZ> points;
      points.push_back(pcl::PointXYZ(x,.05f,.05f));
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(points, *msg);
      msg->header.frame_id="lidar";
      msg->header.stamp.sec=1; msg->header.stamp.nanosec=500'000'000;
      return msg;
    };
    map.segmentedCloudCallback(pairCloud(1.05f), false);
    require(map.lastIntegratedCloudStampNs()==0, "first half alone must not integrate");
    map.segmentedCloudCallback(pairCloud(2.05f), true);
    require(map.lastIntegratedCloudStampNs()==1'500'000'000LL, "matched pair must integrate as one batch");
    require(map.mp_.map_origin_idx_ == latest_origin, "delayed cloud must not slide map window backward");
    require((map.md_.ray_pos_-latest_pose).norm()<1e-9, "latest sensor pose must be restored after historical integration");
    auto empty = pairCloud(0.f);
    empty->width=0; empty->height=0; empty->data.clear(); empty->row_step=0;
    empty->header.stamp.sec=2; empty->header.stamp.nanosec=500'000'000;
    auto ground_only = pairCloud(1.05f);
    ground_only->header.stamp=empty->header.stamp;
    map.segmentedCloudCallback(empty, false);
    map.segmentedCloudCallback(ground_only, true);
    require(map.lastIntegratedCloudStampNs()==2'500'000'000LL,
            "a matched frame with empty nonground and nonempty ground must integrate");

    // A complete pair may arrive before its later odometry sample. Queue
    // pressure should discard an incomplete frame before that usable pair.
    map.cloud_pairs_.clear();
    map.sensor_poses_.clear();
    map.pair_queue_depth_ = 2;
    auto delayed_obstacle = pairCloud(1.05f);
    delayed_obstacle->header.stamp.sec = 4;
    auto delayed_ground = pairCloud(2.05f);
    delayed_ground->header.stamp = delayed_obstacle->header.stamp;
    map.segmentedCloudCallback(delayed_obstacle, false);
    map.segmentedCloudCallback(delayed_ground, true);
    require(map.cloud_pairs_.count(4'500'000'000LL)==1,
            "complete pair must wait for a future odometry bracket");
    map.segmentedCloudCallback(cloud(5,"lidar"), true);
    map.segmentedCloudCallback(cloud(6,"lidar"), false);
    require(map.cloud_pairs_.size()==2 && map.cloud_pairs_.count(4'500'000'000LL)==1 &&
            map.cloud_pairs_.count(5'000'000'000LL)==0,
            "queue pressure must preserve a complete pair before incomplete frames");
    map.sensor_poses_[4'000'000'000LL] = {latest_pose, Eigen::Quaterniond::Identity()};
    map.sensor_poses_[5'000'000'000LL] = {latest_pose, Eigen::Quaterniond::Identity()};
    map.consumeCloudPairs();
    require(map.lastIntegratedCloudStampNs()==4'500'000'000LL,
            "future odometry must integrate the delayed complete pair");
    auto obsolete_obstacle = pairCloud(1.05f);
    obsolete_obstacle->header.stamp.sec = 3;
    auto obsolete_ground = pairCloud(2.05f);
    obsolete_ground->header.stamp = obsolete_obstacle->header.stamp;
    map.segmentedCloudCallback(obsolete_obstacle, false);
    map.segmentedCloudCallback(obsolete_ground, true);
    require(map.cloud_pairs_.count(3'500'000'000LL)==0,
            "pair older than the oldest available pose must be discarded");
    std::cout << "segmented cloud raycast/pair/pose tests passed\n";
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { GridMapSegmentedTest::run(); } catch(const std::exception &e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
