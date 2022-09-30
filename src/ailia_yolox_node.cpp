#include <ailia_yolox/ailia_yolox_node.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>

namespace ailia_yolox
{
AiliaYoloXNode::AiliaYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("ailia_yolox", node_options)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;
  const static std::map<std::string,int> algo_list{
    {"yolox", AILIA_DETECTOR_ALGORITHM_YOLOX},
    {"yolov4", AILIA_DETECTOR_ALGORITHM_YOLOV4},
  };

  std::string model_path = declare_parameter("model_path", "");
  std::string weight_path = declare_parameter("weight_path", "");
  std::string label_path = declare_parameter("label_path", "");
  std::string precision = declare_parameter("precision", "fp32");
  std::string algorithm = declare_parameter("algorithm", "");

  if(algo_list.find(algorithm) == algo_list.end()){
      auto iter = algo_list.begin();
      std::string keys = iter->first;
      iter++;
      for (; iter != algo_list.end(); iter++) {
        keys += ", " + iter->first;
      }
      RCLCPP_ERROR(this->get_logger(), "Undefined algorithm \"%s\", Please select from [%s]", algorithm.c_str(), keys.c_str());
      rclcpp::shutdown();
      return;
  }
  const int algorithm_ = algo_list.at(algorithm);

  RCLCPP_INFO(this->get_logger(), "model_path: %s", model_path.c_str());
  RCLCPP_INFO(this->get_logger(), "weight_path: %s", weight_path.c_str());
  RCLCPP_INFO(this->get_logger(), "label_path: %s", label_path.c_str());
  RCLCPP_INFO(this->get_logger(), "precision: %s", precision.c_str());
  RCLCPP_INFO(this->get_logger(), "algorithm: %s", algorithm.c_str());

  if (!readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
    rclcpp::shutdown();
  }

  ailia_yolox_ = std::make_unique<ailia_yolox::AiliaYoloX>(model_path, weight_path, precision, label_map_.size(), algorithm_, AILIA_ENVIRONMENT_ID_AUTO);
  RCLCPP_INFO(this->get_logger(), "Ailia version: %s", ailia_yolox_->GetAiliaVersion().c_str());
  if(ailia_yolox_->HasError()){
    RCLCPP_ERROR(this->get_logger(), "Ailia initialize failed.");
    RCLCPP_ERROR(this->get_logger(), "%s", ailia_yolox_->GetErrorDescription().c_str());
  }
  RCLCPP_INFO(this->get_logger(), "%s", ailia_yolox_->GetEnvironmentName().c_str());

  timer_ = rclcpp::create_timer(
    this, get_clock(), 1000ms, std::bind(&AiliaYoloXNode::onConnect, this));

  objects_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  image_pub_ = image_transport::create_publisher(this, "~/out/image");
}

void AiliaYoloXNode::onConnect()
{
  using std::placeholders::_1;
  if (objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0 &&
    image_pub_.getNumSubscribers() == 0)
  {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&AiliaYoloXNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data
    );
  }
}

void AiliaYoloXNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  ailia_yolox::ObjectArrays objects;
  if (!ailia_yolox_->doInference({in_image_ptr->image}, objects)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    RCLCPP_WARN(this->get_logger(), "%s", ailia_yolox_->GetErrorDescription().c_str());
    return;
  }
  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.classification.emplace_back(
      autoware_auto_perception_msgs::build<Label>()
      .label(Label::UNKNOWN)
      .probability(yolox_object.score));
    if (label_map_[yolox_object.type] == "CAR") {
      object.object.classification.front().label = Label::CAR;
    } else if (label_map_[yolox_object.type] == "PEDESTRIAN") {
      object.object.classification.front().label = Label::PEDESTRIAN;
    } else if (label_map_[yolox_object.type] == "BUS") {
      object.object.classification.front().label = Label::BUS;
    } else if (label_map_[yolox_object.type] == "TRUCK") {
      object.object.classification.front().label = Label::TRUCK;
    } else if (label_map_[yolox_object.type] == "BICYCLE") {
      object.object.classification.front().label = Label::BICYCLE;
    } else if (label_map_[yolox_object.type] == "MOTORCYCLE") {
      object.object.classification.front().label = Label::MOTORCYCLE;
    }
    out_objects.feature_objects.push_back(object);
    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
  }
  image_pub_.publish(in_image_ptr->toImageMsg());

  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);
}

bool AiliaYoloXNode::readLabelFile(const std::string & label_path)
{
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", label_path.c_str());
    return false;
  }
  int label_index{};
  std::string label;
  while (getline(label_file, label)) {
    label_map_.insert({label_index, label});
    ++label_index;
  }
  return true;
}

}  // namespace ailia_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ailia_yolox::AiliaYoloXNode)
