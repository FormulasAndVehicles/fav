#include "scenario_node.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/param_utils.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
namespace fav {
namespace scenario {

static constexpr double kDepth = -0.5;

ScenarioNode::ScenarioNode() : Node("scenario_node") {
  HIPPO_COMMON_DECLARE_PARAM_READONLY(scenario);
  if (!ReadConfig(params_.scenario)) {
    RCLCPP_FATAL(get_logger(), "Failed to load config.");
    return;
  }
  PopulateViewpointsMessage();

  InitPublishers();
  InitClients();
  InitSubscriptions();
  InitServices();
  InitTimers();
}

void ScenarioNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);

  topic = "obstacles";
  obstacles_pub_ =
      create_publisher<scenario_msgs::msg::PolygonsStamped>(topic, qos);

  topic = "viewpoints";
  viewpoints_pub_ =
      create_publisher<scenario_msgs::msg::Viewpoints>(topic, qos);

  topic = "~/viewpoint_progress";
  progress_pub_ = create_publisher<std_msgs::msg::Float32>(topic, qos);

  topic = "~/overlay_t_finish_text";
  finish_time_overlay_pub_ =
      create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(topic, qos);

  topic = "~/overlay_status_text";
  status_overlay_pub_ =
      create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(topic, qos);

  topic = "~/marker_array";
  marker_array_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(topic, qos);
}

void ScenarioNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "vision_pose_cov";
  using geometry_msgs::msg::PoseWithCovarianceStamped;
  pose_sub_ = create_subscription<PoseWithCovarianceStamped>(
      topic, qos,
      [this](const PoseWithCovarianceStamped::SharedPtr msg) { OnPose(msg); });
}

void ScenarioNode::InitTimers() {
  timer_ = create_timer(std::chrono::milliseconds(20), [this]() { OnTimer(); });
}

void ScenarioNode::InitClients() {
  std::string name;
  client_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::QoS qos = rclcpp::ServicesQoS();

  name = "path_planner/start";
  start_path_planner_client_ =
      create_client<std_srvs::srv::Trigger>(name, qos, client_cb_group_);

  name = "path_planner/move_to_start";
  move_to_start_client_ = create_client<scenario_msgs::srv::MoveToStart>(
      name, qos, client_cb_group_);

  name = "path_planner/stop";
  stop_path_planner_client_ =
      create_client<std_srvs::srv::Trigger>(name, qos, client_cb_group_);
}

void ScenarioNode::InitServices() {
  std::string name;
  using std_srvs::srv::Trigger;

  name = "~/start";
  start_service_ = create_service<Trigger>(
      name, [this](const Trigger::Request::SharedPtr request,
                   Trigger::Response::SharedPtr response) {
        ServeStart(request, response);
      });

  name = "~/reset";
  reset_service_ = create_service<Trigger>(
      name, [this](const Trigger::Request::SharedPtr request,
                   Trigger::Response::SharedPtr response) {
        ServeReset(request, response);
      });
}

void ScenarioNode::ServeStart(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (running_) {
    response->success = false;
    response->message = "Already running.";
    return;
  }
  Reset();
  running_ = true;
  using hippo_common::convert::EigenToRos;
  auto req = std::make_shared<scenario_msgs::srv::MoveToStart_Request>();
  EigenToRos(vehicle_position_, req->current_pose.position);
  EigenToRos(vehicle_orientation_, req->current_pose.orientation);
  req->target_pose = viewpoints_msg_.viewpoints.at(0).pose;
  if (!move_to_start_client_) {
    RCLCPP_ERROR(get_logger(), "move_to_start service client not initialized");
    return;
  }
  auto future = move_to_start_client_->async_send_request(req);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::timeout) {
    // we have to remove pending service calls manually according to the docs.
    move_to_start_client_->remove_pending_request(future);
    RCLCPP_WARN(get_logger(),
                "Failed to make a call to the path_planner/move_to_start "
                "service. The scenario will start nonetheless. But the "
                "path_planner will probably not work.");
  }
  response->success = true;
  response->message = "Started";
}
void ScenarioNode::ServeReset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  Reset();
  auto req = std::make_shared<std_srvs::srv::Trigger_Request>();
  if (!stop_path_planner_client_) {
    RCLCPP_ERROR(get_logger(),
                 "Service for starting the path planner not initialied.");
    return;
  }
  auto future = stop_path_planner_client_->async_send_request(req);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::timeout) {
    stop_path_planner_client_->remove_pending_request(future);
    RCLCPP_WARN(get_logger(),
                "Failed to make a call to the path_planner/stop service. The "
                "scenario will start nonetheless. But the path_planner will "
                "probably not work.:");
  }
  response->success = true;
  response->message = "Reset";
}

void ScenarioNode::PublishProgressGauge() {
  int i = viewpoint_index_.current < 0 ? viewpoint_index_.previous
                                       : viewpoint_index_.current;
  std_msgs::msg::Float32 msg;
  if (i >= 0 && !viewpoints_.at(i).IsCompleted()) {
    msg.data = viewpoints_.at(i).Progress();
  }
  progress_pub_->publish(msg);
}

void ScenarioNode::PublishFinishTimeOverlay(double _dt, bool delete_text) {
  rviz_2d_overlay_msgs::msg::OverlayText msg;
  msg.action = delete_text ? msg.DELETE : msg.ADD;
  msg.horizontal_alignment = msg.LEFT;
  msg.horizontal_distance = 10;
  msg.vertical_alignment = msg.TOP;
  msg.vertical_distance = 40;
  msg.fg_color.a = 1.0;
  msg.fg_color.g = 1.0;
  msg.line_width = 10;
  msg.text_size = 18.0;
  msg.width = 1000;
  msg.height = 50;

  char buffer[256];
  snprintf(buffer, sizeof(buffer), "Finished after %.2lf", _dt);
  msg.text = buffer;
  finish_time_overlay_pub_->publish(msg);
}

void ScenarioNode::PublishStatusOverlay() {
  rviz_2d_overlay_msgs::msg::OverlayText msg;
  msg.horizontal_alignment = msg.LEFT;
  msg.horizontal_distance = 10;
  msg.vertical_alignment = msg.TOP;
  msg.vertical_distance = 10;
  msg.fg_color.a = 1.0;
  msg.fg_color.g = 1.0;
  msg.line_width = 10;
  msg.text_size = 18.0;
  msg.width = 1000;
  msg.height = 50;
  if (running_) {
    msg.fg_color.r = 1.0;
    msg.fg_color.g = 1.0;
    msg.fg_color.b = 0.0;
    if (start_position_reached_) {
      double dt = (now() - t_start_).nanoseconds() * 1e-9;
      char buffer[256];
      snprintf(buffer, sizeof(buffer), "Time: %.2lf", dt);
      msg.text = buffer;
    } else {
      msg.text = "Moving to start position";
    }
  } else {
    msg.fg_color.r = 1.0;
    msg.fg_color.g = 0.0;
    msg.fg_color.b = 0.0;
    msg.text = "Not running";
  }
  status_overlay_pub_->publish(msg);
}

void ScenarioNode::PublishMarkerArray() {
  marker_array_.markers.clear();
  AddObstacleMarkers();
  AddViewpointMarkers();
  if (!marker_array_pub_) {
    RCLCPP_ERROR(get_logger(), "Marker array publisher not initialized.");
    return;
  }
  marker_array_pub_->publish(marker_array_);
}

void ScenarioNode::AddObstacleMarkers() {
  visualization_msgs::msg::Marker marker;
  int i = 0;
  marker.header.stamp = now();
  marker.header.frame_id = "map";
  for (const auto &obstacle : obstacles_) {
    marker.points.clear();
    for (const auto &point : obstacle.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      marker.points.push_back(p);
    }
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    if (!running_) {
      marker.action = marker.DELETEALL;
    }
    marker.id = i++;
    marker.ns = "obstacles";
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker_array_.markers.push_back(marker);
  }
}

void ScenarioNode::AddViewpointMarkers() {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = "map";
  marker.type = marker.ARROW;
  marker.ns = "viewpoints";
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  int i = 0;
  const auto t_now = now();
  for (const auto &viewpoint : viewpoints_msg_.viewpoints) {
    marker.pose = viewpoint.pose;
    marker.action = marker.ADD;
    marker.id = i++;
    if (!running_) {
      marker.action = marker.DELETEALL;
    }
    marker.color.r = 1.0 - viewpoint.completed;
    marker.color.g = 1.0 * viewpoint.completed;
    marker_array_.markers.push_back(marker);
    if (i == viewpoint_index_.current && !viewpoint.completed) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
  }
}

void ScenarioNode::OnPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg) {
  using hippo_common::convert::RosToEigen;
  RosToEigen(_msg->pose.pose.position, vehicle_position_);
  RosToEigen(_msg->pose.pose.orientation, vehicle_orientation_);

  UpdateViewpointProgress();
  viewpoint_index_.current = FindViewpointInTolerance();
  PopulateViewpointsMessage();
  if (viewpoint_index_.current >= 0) {
    viewpoint_index_.previous = viewpoint_index_.current;
  }

  PublishProgressGauge();
  if (running_ && viewpoints_finished_) {
    running_ = false;
    double dt = (now() - t_start_).nanoseconds() * 1e-9;
    PublishFinishTimeOverlay(dt);
    RCLCPP_INFO(get_logger(), "Finished after %.2lfs", dt);
  }
}

void ScenarioNode::UpdateViewpointProgress() {
  if (!viewpoints_.at(0).IsCompleted()) {
    // still moving to start location. no progress of other viewpoints allowed
    viewpoints_[0].Update(vehicle_position_, vehicle_orientation_);
    if (viewpoints_[0].IsCompleted()) {
      start_position_reached_ = true;
      t_start_ = now();
      auto req = std::make_shared<std_srvs::srv::Trigger_Request>();
      auto future = start_path_planner_client_->async_send_request(req);
      auto status = future.wait_for(std::chrono::milliseconds(500));
      if (status == std::future_status::timeout) {
        RCLCPP_WARN(
            get_logger(),
            "Could not call path_planner/start service. Scenario will continue "
            "nonetheless. But probably things aren't working.");
      }
    }
    return;
  }
  bool progress_disabled{false};
  viewpoints_finished_ = true;
  for (auto &viewpoint : viewpoints_) {
    if (viewpoint.IsCompleted()) {
      continue;
    }
    viewpoints_finished_ = false;
    if (progress_disabled) {
      viewpoint.Update(false);
    } else {
      viewpoint.Update(vehicle_position_, vehicle_orientation_);
      if (viewpoint.IsInTolerance()) {
        // only let the first viewpoint in tolerance continue its progress.
        progress_disabled = true;
      }
    }
  }
}

int ScenarioNode::FindViewpointInTolerance() {
  for (size_t i = 0; i < viewpoints_.size(); ++i) {
    if (viewpoints_.at(i).IsCompleted()) {
      continue;
    }
    if (viewpoints_.at(i).IsInTolerance()) {
      return i;
    }
  }
  return -1;
}

void ScenarioNode::OnTimer() {
  scenario_msgs::msg::PolygonsStamped obstacles_msg;
  obstacles_msg.header.stamp = now();
  obstacles_msg.header.frame_id = "map";
  obstacles_msg.polygons = obstacles_;

  if (running_) {
    if (obstacles_pub_) {
      obstacles_pub_->publish(obstacles_msg);
    }
    if (viewpoints_pub_) {
      viewpoints_msg_.header.stamp = now();
      viewpoints_msg_.header.frame_id = "map";
      viewpoints_pub_->publish(viewpoints_msg_);
    }
  }
  PublishStatusOverlay();
  PublishMarkerArray();
}

void ScenarioNode::Reset() {
  for (auto &viewpoint : viewpoints_) {
    viewpoint.Reset();
  }
  PopulateViewpointsMessage();
  running_ = false;
  start_position_reached_ = false;
  viewpoints_finished_ = false;
  // TODO delete the finish time overlay
}

void ScenarioNode::PopulateViewpointsMessage() {
  using hippo_common::convert::EigenToRos;
  viewpoints_msg_.viewpoints.clear();
  for (auto const &viewpoint : viewpoints_) {
    scenario_msgs::msg::Viewpoint viewpoint_msg;
    geometry_msgs::msg::Pose p;
    EigenToRos(viewpoint.position, p.position);
    EigenToRos(viewpoint.orientation, p.orientation);

    viewpoint_msg.pose = p;
    viewpoint_msg.progress = viewpoint.Progress();
    viewpoint_msg.completed = viewpoint.IsCompleted();
    viewpoints_msg_.viewpoints.push_back(viewpoint_msg);
  }
}

std::optional<std::string> ScenarioNode::GetScenarioFilePath(int scenario) {
  std::string file_path;
  std::string pkg{"final_project"};
  try {
    file_path = ament_index_cpp::get_package_share_directory(pkg);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    RCLCPP_FATAL(
        get_logger(),
        "Failed to load scenario config because [%s] could not be found.",
        pkg.c_str());
    return std::nullopt;
  }
  file_path += "/config/scenario_" + std::to_string(scenario) + ".yaml";
  return file_path;
}

bool ScenarioNode::ReadConfig(int scenario) {
  std::optional<std::string> file_path = GetScenarioFilePath(scenario);
  if (!file_path) {
    return false;
  }
  YAML::Node node;
  try {
    node = YAML::LoadFile(*file_path);
  } catch (const YAML::BadFile &) {
    RCLCPP_FATAL(
        get_logger(),
        "Could not load scenario config because file [%s] could not be "
        "loaded. Does it exist?",
        file_path->c_str());
    return false;
  } catch (const YAML::ParserException &) {
    RCLCPP_FATAL(
        get_logger(),
        "Could not load scenario config from [%s] because it is malformed.",
        file_path->c_str());
    return false;
  }
  ReadViewpoints(node);
  ReadObstacles(node);
  return true;
}

bool ScenarioNode::ReadObstacles(const YAML::Node &node) {
  using geometry_msgs::msg::Point32;
  using geometry_msgs::msg::Polygon;
  obstacles_.clear();
  for (const YAML::Node &obstacle : node["obstacles"]) {
    std::vector<Point32> corners;
    Polygon polygon;
    for (const YAML::Node &corner : obstacle["corners"]) {
      std::vector<double> corner_points = corner.as<std::vector<double>>();
      Point32 p;
      p.x = corner_points.at(0);
      p.y = corner_points.at(1);
      p.z = kDepth;
      polygon.points.push_back(p);
    }
    polygon.points.push_back(polygon.points.at(0));
    obstacles_.push_back(polygon);
  }
  return true;
}

bool ScenarioNode::ReadViewpoints(const YAML::Node &node) {
  viewpoint_poses_ = node["viewpoints"].as<hippo_common::yaml::Poses>();
  viewpoints_.clear();
  for (const auto &pose : viewpoint_poses_) {
    viewpoints_.emplace_back(pose.position, pose.orientation, 0.1, 0.2);
  }
  return true;
}
}  // namespace scenario
}  // namespace fav

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::vector<std::shared_ptr<std::thread>> spin_threads;
  rclcpp::experimental::executors::EventsExecutor exec_main;
  rclcpp::experimental::executors::EventsExecutor exec_client;
  auto node = std::make_shared<fav::scenario::ScenarioNode>();
  exec_main.add_node(node->get_node_base_interface());
  exec_client.add_callback_group(node->client_cb_group_,
                                 node->get_node_base_interface());
  spin_threads.push_back(
      std::make_shared<std::thread>([&exec_main]() { exec_main.spin(); }));
  spin_threads.push_back(
      std::make_shared<std::thread>([&exec_client]() { exec_client.spin(); }));

  for (auto &thread : spin_threads) {
    thread->join();
  }
  rclcpp::shutdown();
  return 0;
}
