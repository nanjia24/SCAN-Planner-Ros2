#include <traj_utils/planning_visualization.h>
#include <cmath>
#include <limits>
#include <std_msgs/msg/color_rgba.hpp>

using std::cout;
using std::endl;
namespace scan_planner
{
  vector<Eigen::Vector3d> PlanningVisualization::resamplePathByArcLength(
      const vector<Eigen::Vector3d> &points, const double spacing)
  {
    if (points.size() <= 1 || spacing <= 1e-6)
      return points;

    vector<double> cumulative(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i)
      cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).norm();

    const double total_length = cumulative.back();
    if (total_length <= 1e-6)
      return {points.front()};

    const int interval_count = std::max(
        1, static_cast<int>(std::floor(total_length / spacing + 0.5)));
    const double actual_spacing = total_length / interval_count;
    vector<Eigen::Vector3d> sampled;
    sampled.reserve(interval_count + 1);

    size_t upper_index = 1;
    sampled.push_back(points.front());
    for (int sample_index = 1; sample_index < interval_count; ++sample_index)
    {
      const double target = actual_spacing * sample_index;
      while (upper_index + 1 < cumulative.size() && cumulative[upper_index] < target)
        ++upper_index;

      const size_t lower_index = upper_index - 1;
      const double segment_length = cumulative[upper_index] - cumulative[lower_index];
      if (segment_length <= 1e-9)
      {
        sampled.push_back(points[upper_index]);
        continue;
      }

      const double ratio = (target - cumulative[lower_index]) / segment_length;
      sampled.push_back(points[lower_index] + ratio * (points[upper_index] - points[lower_index]));
    }
    sampled.push_back(points.back());
    return sampled;
  }

  PlanningVisualization::PlanningVisualization(rclcpp::Node *node)
  {
    node_ = node;
    if (!node_->has_parameter("visualization.frame_id"))
      node_->declare_parameter<std::string>("visualization.frame_id", "world");
    frame_id_ = node_->get_parameter("visualization.frame_id").as_string();

    const auto marker_qos = rclcpp::QoS(20).reliable().transient_local();
    goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>("goal_point", marker_qos);
    global_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("global_list", marker_qos);
    init_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("init_list", marker_qos);
    optimal_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("optimal_list", marker_qos);
    a_star_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("a_star_list", marker_qos);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    pub->publish(sphere);
    pub->publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = node_->now();
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::msg::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame_id_;
    sphere.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub->publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id)
  {
    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(
        global_list_pub,
        resamplePathByArcLength(global_pts, 0.4),
        scale,
        color,
        id);
  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayOptimalTraj(UniformBspline position_traj, int id)
  {
    const double duration = position_traj.getTimeSum();
    if (duration < 1e-6)
    {
      return;
    }

    UniformBspline velocity_traj = position_traj.getDerivative();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(duration / 0.1)) + 1);

    std::vector<Eigen::Vector3d> points;
    std::vector<double> speeds;
    points.reserve(sample_num);
    speeds.reserve(sample_num);

    double min_speed = std::numeric_limits<double>::max();
    double max_speed = 0.0;
    for (int i = 0; i < sample_num; ++i)
    {
      const double t = duration * static_cast<double>(i) / static_cast<double>(sample_num - 1);
      Eigen::Vector3d pos = position_traj.evaluateDeBoorT(t);
      const double speed = velocity_traj.evaluateDeBoorT(t).norm();

      points.push_back(pos);
      speeds.push_back(speed);
      min_speed = std::min(min_speed, speed);
      max_speed = std::max(max_speed, speed);
    }

    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08;
    line_strip.scale.x = 0.08;

    const double speed_range = std::max(1e-6, max_speed - min_speed);
    for (size_t i = 0; i < points.size(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

      const double ratio = std::min(1.0, std::max(0.0, (speeds[i] - min_speed) / speed_range));
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0;
      color.r = 1.0;
      color.g = 1.0 - ratio;
      color.b = 0.0;
      sphere.colors.push_back(color);
      line_strip.colors.push_back(color);
    }

    optimal_list_pub->publish(sphere);
    optimal_list_pub->publish(line_strip);
  }

  void PlanningVisualization::clearOptimalTraj(const int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.id = id;
    optimal_list_pub->publish(marker);
    marker.id = id + 1000;
    optimal_list_pub->publish(marker);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star paths different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    // for ( int i=0; i<10; i++ )
    // {
    //   //Eigen::Vector4d color(1,1,0,0);
    //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
    // }

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(const MarkerArrayPublisher::SharedPtr &pub,
                                                const vector<Eigen::Vector3d> &list,
                                                double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::msg::MarkerArray array;
    // clear
    pub->publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub->publish(array);
  }

  // PlanningVisualization::
} // namespace scan_planner
