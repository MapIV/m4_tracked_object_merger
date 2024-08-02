#include "tracked_object_merger.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tracked_object_merger_utils.hpp"
#include <cmath>

namespace tracked_object_merger
{

TrackedObjectMergerNode::TrackedObjectMergerNode(const rclcpp::NodeOptions & options)
: Node("tracked_object_merger_node", options)
{
  sub_main_objects_ = create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "input/main_object", rclcpp::QoS{1},
    std::bind(&TrackedObjectMergerNode::mainObjectsCallback, this, std::placeholders::_1));
  sub_sub_objects_ = create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "input/sub_object", rclcpp::QoS{1},
    std::bind(&TrackedObjectMergerNode::subObjectsCallback, this, std::placeholders::_1));
  merged_object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>("output/object", rclcpp::QoS{1});

  // Load parameters
  time_sync_threshold_ = declare_parameter<double>("time_sync_threshold", 0.1);
  sub_object_timeout_sec_ = declare_parameter<double>("sub_object_timeout_sec", 1.0);

  // Initialize can_assign_matrix
  const auto can_assign_vector_tmp = declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  std::vector<int> can_assign_vector(can_assign_vector_tmp.begin(), can_assign_vector_tmp.end());
  const int label_num = static_cast<int>(std::sqrt(can_assign_vector.size()));
  can_assign_matrix_ = Eigen::Map<Eigen::MatrixXi>(can_assign_vector.data(), label_num, label_num).transpose();

  // Initialize distance_threshold_matrix
  const auto distance_threshold_vector_tmp = declare_parameter<std::vector<double>>("distance_threshold_matrix");
  std::vector<double> distance_threshold_vector(distance_threshold_vector_tmp.begin(), distance_threshold_vector_tmp.end());
  distance_threshold_matrix_ = Eigen::Map<Eigen::MatrixXd>(distance_threshold_vector.data(), label_num, label_num).transpose();
}

void TrackedObjectMergerNode::mainObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg)
{
  main_objects_ = msg;
  mergeAndPublishObjects();
}

void TrackedObjectMergerNode::subObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg)
{
  sub_objects_ = msg;
}

void TrackedObjectMergerNode::mergeAndPublishObjects()
{
  if (!main_objects_) {
    return;
  }

  autoware_auto_perception_msgs::msg::TrackedObjects merged_objects;
  merged_objects.header = main_objects_->header;

  // Check if sub_objects_ is available and not too old
  if (sub_objects_ && (rclcpp::Time(main_objects_->header.stamp) - rclcpp::Time(sub_objects_->header.stamp)).seconds() <= time_sync_threshold_) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Merging main and sub objects");

    // Merge main and sub objects
    for (const auto & main_obj : main_objects_->objects) {
      bool merged = false;
      for (const auto & sub_obj : sub_objects_->objects) {
        if (isClose(main_obj, sub_obj) && areCompatible(main_obj, sub_obj)) {
          merged_objects.objects.push_back(main_obj);
          merged = true;
          break;
        }
      }
      if (!merged) {
        merged_objects.objects.push_back(main_obj);
      }
    }

    for (const auto & sub_obj : sub_objects_->objects) {
      bool already_merged = false;
      for (const auto & main_obj : merged_objects.objects) {
        if (isClose(main_obj, sub_obj)) {
          already_merged = true;
          break;
        }
      }
      if (!already_merged) {
        merged_objects.objects.push_back(sub_obj);
      }
    }
  } else {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Only main objects were published");
    // No valid sub_objects_, just publish main_objects_
    merged_objects.objects = main_objects_->objects;
  }

  merged_object_pub_->publish(merged_objects);
}

bool TrackedObjectMergerNode::isClose(const autoware_auto_perception_msgs::msg::TrackedObject & obj1, const autoware_auto_perception_msgs::msg::TrackedObject & obj2)
{
  int label1 = object_recognition_utils::getHighestProbLabel(obj1.classification);
  int label2 = object_recognition_utils::getHighestProbLabel(obj2.classification);
  double distance_threshold = distance_threshold_matrix_(label1, label2);

  const auto & pos1 = obj1.kinematics.pose_with_covariance.pose.position;
  const auto & pos2 = obj2.kinematics.pose_with_covariance.pose.position;
  double distance = std::sqrt(
    std::pow(pos1.x - pos2.x, 2) + std::pow(pos1.y - pos2.y, 2));
  return distance < distance_threshold;
}

bool TrackedObjectMergerNode::areCompatible(const autoware_auto_perception_msgs::msg::TrackedObject & obj1, const autoware_auto_perception_msgs::msg::TrackedObject & obj2)
{
  int label1 = object_recognition_utils::getHighestProbLabel(obj1.classification);
  int label2 = object_recognition_utils::getHighestProbLabel(obj2.classification);
  return can_assign_matrix_(label1, label2);
}

std::optional<autoware_auto_perception_msgs::msg::TrackedObjects> TrackedObjectMergerNode::interpolateObjectState(
  const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr & sub_objects, const std_msgs::msg::Header & output_header)
{
  if (!sub_objects) {
    return std::nullopt;
  }

  autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr closest_time_sub_objects;
  autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr closest_time_sub_objects_later;

  for (const auto & sub_object : sub_objects_buffer_) {
    if (getUnixTime(sub_object->header) < getUnixTime(output_header)) {
      closest_time_sub_objects = sub_object;
    } else {
      closest_time_sub_objects_later = sub_object;
      break;
    }
  }

  if (!closest_time_sub_objects && !closest_time_sub_objects_later) {
    return std::nullopt;
  }

  if (!closest_time_sub_objects) {
    if ((rclcpp::Time(closest_time_sub_objects_later->header.stamp) - rclcpp::Time(output_header.stamp)).seconds() > time_sync_threshold_) {
      return std::nullopt;
    }
    return *closest_time_sub_objects_later;
  }

  if (!closest_time_sub_objects_later) {
    const auto dt = (rclcpp::Time(output_header.stamp) - rclcpp::Time(closest_time_sub_objects->header.stamp)).seconds();
    if (dt > time_sync_threshold_) {
      return std::nullopt;
    }
    return *closest_time_sub_objects;
  }

  return tracked_object_merger_utils::interpolateTrackedObjects(*closest_time_sub_objects, *closest_time_sub_objects_later, output_header);
}

double TrackedObjectMergerNode::getUnixTime(const std_msgs::msg::Header & header)
{
  return header.stamp.sec + header.stamp.nanosec * 1e-9;
}

}  // namespace tracked_object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tracked_object_merger::TrackedObjectMergerNode)
