
#ifndef AILIA_YOLOX__AILIA_YOLOX_NODE_HPP_
#define AILIA_YOLOX__AILIA_YOLOX_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ailia_yolox/ailia_yolox.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <X11/Xlib.h>

namespace ailia_yolox
{
class AiliaYoloXNode : public rclcpp::Node
{
public:
  explicit AiliaYoloXNode(const rclcpp::NodeOptions & node_options);

private:
  void onConnect();
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  bool readLabelFile(const std::string & label_path);

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    objects_pub_;

  image_transport::Subscriber image_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<int, std::string> label_map_;
  std::unique_ptr<ailia_yolox::AiliaYoloX> ailia_yolox_;
};

}  // namespace ailia_yolox

#endif  // AILIA_YOLOX__AILIA_YOLOX_NODE_HPP_
