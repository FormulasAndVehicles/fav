#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hippo_common/yaml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <scenario_msgs/msg/polygons_stamped.hpp>
#include <scenario_msgs/msg/viewpoints.hpp>
#include <scenario_msgs/srv/move_to_start.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "viewpoint.hpp"

namespace fav {
namespace scenario {

class ScenarioNode : public rclcpp::Node {
 public:
  ScenarioNode();

 private:
  struct ViewpointIndex {
    int current{-1};
    int previous{-1};
  };
  struct Params {
    int scenario;
  };
  void InitPublishers();
  void InitSubscriptions();
  void InitTimers();
  void InitClients();
  void InitServices();
  void OnTimer();
  void PopulateViewpointsMessage();
  void Reset();
  void OnPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void ServeStart(const std_srvs::srv::Trigger_Request::SharedPtr,
                  std_srvs::srv::Trigger_Response::SharedPtr);
  void ServeReset(const std_srvs::srv::Trigger_Request::SharedPtr,
                  std_srvs::srv::Trigger_Response::SharedPtr);
  bool ReadConfig(int scenario);
  std::optional<std::string> GetScenarioFilePath(int scenario);
  bool ReadObstacles(const YAML::Node &node);
  bool ReadViewpoints(const YAML::Node &node);
  int FindViewpointInTolerance();
  void UpdateViewpointProgress();
  void PublishProgressGauge();
  void PublishFinishTimeOverlay(double dt, bool delete_text = false);
  void PublishStatusOverlay();
  void PublishMarkerArray();
  void AddObstacleMarkers();
  void AddViewpointMarkers();

  scenario_msgs::msg::Viewpoints viewpoints_msg_;
  std::vector<Viewpoint> viewpoints_;
  hippo_common::yaml::Poses viewpoint_poses_;
  std::vector<geometry_msgs::msg::Polygon> obstacles_;
  bool running_{false};
  bool start_position_reached_{false};
  bool viewpoints_finished_{false};
  Eigen::Vector3d vehicle_position_;
  Eigen::Quaterniond vehicle_orientation_;
  ViewpointIndex viewpoint_index_;
  rclcpp::Time t_start_;
  visualization_msgs::msg::MarkerArray marker_array_;
  Params params_;

  rclcpp::Publisher<scenario_msgs::msg::PolygonsStamped>::SharedPtr
      obstacles_pub_;
  rclcpp::Publisher<scenario_msgs::msg::Viewpoints>::SharedPtr viewpoints_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr
      finish_time_overlay_pub_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr
      status_overlay_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

 private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_path_planner_client_;
  rclcpp::Client<scenario_msgs::srv::MoveToStart>::SharedPtr
      move_to_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_path_planner_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

}  // namespace scenario
}  // namespace fav
