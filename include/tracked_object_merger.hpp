#ifndef TRACKED_OBJECT_MERGER_NODE_HPP
#define TRACKED_OBJECT_MERGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include <Eigen/Core>
#include <optional>
#include <unordered_map>
#include <vector>

namespace tracked_object_merger
{

class TrackedObjectMergerNode : public rclcpp::Node
{
public:
  explicit TrackedObjectMergerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr sub_main_objects_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr sub_sub_objects_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr merged_object_pub_;
  autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr main_objects_;
  autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr sub_objects_;
  std::vector<autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr> sub_objects_buffer_;

  double time_sync_threshold_;
  double sub_object_timeout_sec_;
  double distance_threshold_;
  Eigen::MatrixXi can_assign_matrix_;
  Eigen::MatrixXd distance_threshold_matrix_;

  void mainObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg);
  void subObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg);
  void mergeAndPublishObjects();
  bool isClose(const autoware_auto_perception_msgs::msg::TrackedObject & obj1, const autoware_auto_perception_msgs::msg::TrackedObject & obj2);
  bool areCompatible(const autoware_auto_perception_msgs::msg::TrackedObject & obj1, const autoware_auto_perception_msgs::msg::TrackedObject & obj2);
  std::optional<autoware_auto_perception_msgs::msg::TrackedObjects> interpolateObjectState(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr & sub_objects, const std_msgs::msg::Header & output_header);
  double getUnixTime(const std_msgs::msg::Header & header);
};

}  // namespace tracked_object_merger

#endif // TRACKED_OBJECT_MERGER_NODE_HPP
