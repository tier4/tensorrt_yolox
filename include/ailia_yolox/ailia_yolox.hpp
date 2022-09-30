#ifndef AILIA_YOLOX__AILIA_YOLOX_HPP_
#define AILIA_YOLOX__AILIA_YOLOX_HPP_

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

// #include <tensorrt_yolox/tensorrt_yolox.hpp>
#include "ailia.h"
#include "ailia_detector.h"


#define IMAGE_CHANNEL 3

namespace ailia_yolox
{
struct Object
{
  int32_t x_offset;
  int32_t y_offset;
  int32_t height;
  int32_t width;
  float score;
  int32_t type;
};

using ObjectArray = std::vector<Object>;
using ObjectArrays = std::vector<ObjectArray>;

class AiliaYoloX
{
public:
  AiliaYoloX(
    const std::string & model_path, 
    const std::string & weight_path, 
    const std::string & precision, 
    const unsigned int labelNum,
    const int algorithm,
    const int env_id = AILIA_ENVIRONMENT_ID_AUTO);
  ~AiliaYoloX();
  
  std::string GetAiliaVersion();
  std::string GetEnvironmentName();
  std::vector<std::string> GetAllEnvironmentNames();

  bool doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects);

  bool HasError();
  std::string GetErrorDescription();

private:
  AILIANetwork *ailia;
  AILIADetector *detector;
  std::string error_msg;
  bool error_;
  const int algorithm_;

  void Preprocess(const cv::Mat& image, cv::Mat& out_image, float& out_scale);
  bool CheckError(const int status, const std::string& funcName);
};

}  // namespace ailia_yolox

#endif  // AILIA_YOLOX__AILIA_YOLOX_HPP_
