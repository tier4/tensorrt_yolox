#include <ailia_yolox/ailia_yolox.hpp>
#include <iostream>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#define IMAGE_WIDTH  416 // Must be a multiple of 32
#define IMAGE_HEIGHT 416
#define THRESHOLD 0.05f
#define IOU       0.45f

#define AILIA_ENVIRONMENT_TYPE_SCAI (8)

namespace ailia_yolox
{
AiliaYoloX::AiliaYoloX(
  const std::string &model_path, 
  const std::string &weight_path, 
  const std::string &precision,
  const unsigned int labelNum,
  const int algorithm,
  const int env_id) : algorithm_(algorithm)
{
  error_ = false;
	unsigned int env_count;
  int status;
	status = ailiaGetEnvironmentCount(&env_count);
  if(CheckError(status, "ailiaGetEnvironmentCount")){
    return;
	}
  int fp = AILIA_ENVIRONMENT_PROPERTY_NORMAL;
  if(precision == "fp16"){
    fp = AILIA_ENVIRONMENT_PROPERTY_FP16;
  }
  int env_id_ = env_id;
  if(env_id_ == AILIA_ENVIRONMENT_ID_AUTO){
    for(unsigned int i = 0; i < env_count; i++){
      AILIAEnvironment *env;
      ailiaGetEnvironment(&env, i, AILIA_ENVIRONMENT_VERSION);
      if(env->type == AILIA_ENVIRONMENT_TYPE_SCAI && env->props == fp){
        env_id_ = env->id;
        break;
      }
    }
  }
  if(env_id_ == AILIA_ENVIRONMENT_ID_AUTO){
    for(unsigned int i = 0; i < env_count; i++){
      AILIAEnvironment *env;
      ailiaGetEnvironment(&env, i, AILIA_ENVIRONMENT_VERSION);
      if(env->type == AILIA_ENVIRONMENT_TYPE_GPU && env->props == fp){
        env_id_ = env->id;
        break;
      }
    }
  }

  status = ailiaCreate(&ailia, env_id_, AILIA_MULTITHREAD_AUTO);
  if(CheckError(status, "ailiaCreate")){
    return;
	}

  status = ailiaOpenStreamFile(ailia, model_path.c_str());
  if(CheckError(status, "ailiaOpenStreamFile")){
    error_msg += "\ninput path: " + model_path + "\n";
    ailiaDestroy(ailia);
    return;
  }

  status = ailiaOpenWeightFile(ailia, weight_path.c_str());
  if(CheckError(status, "ailiaOpenWeightFile")){
    error_msg += "\ninput path: " + weight_path + "\n";
    ailiaDestroy(ailia);
    return;
  }

  const unsigned int flags = AILIA_DETECTOR_FLAG_NORMAL;
  switch(algorithm_){
    case AILIA_DETECTOR_ALGORITHM_YOLOX:
      status = ailiaCreateDetector(&detector, ailia,
                                    AILIA_NETWORK_IMAGE_FORMAT_BGR,
                                    AILIA_NETWORK_IMAGE_CHANNEL_FIRST,
                                    AILIA_NETWORK_IMAGE_RANGE_UNSIGNED_INT8,
                                    AILIA_DETECTOR_ALGORITHM_YOLOX,
                                    labelNum, flags);
      break;
    case AILIA_DETECTOR_ALGORITHM_YOLOV4:
      status = ailiaCreateDetector(&detector, ailia,
                                  AILIA_NETWORK_IMAGE_FORMAT_RGB,
                                  AILIA_NETWORK_IMAGE_CHANNEL_FIRST,
                                  AILIA_NETWORK_IMAGE_RANGE_UNSIGNED_FP32,
                                  AILIA_DETECTOR_ALGORITHM_YOLOV4,
                                  labelNum, flags);
  }
  if(CheckError(status, "ailiaCreateDetector")){
      ailiaDestroy(ailia);
      return;
  }

  if(algorithm_ == AILIA_DETECTOR_ALGORITHM_YOLOX){
    status = ailiaDetectorSetInputShape(detector, IMAGE_WIDTH, IMAGE_HEIGHT);
    if(CheckError(status, "ailiaDetectorSetInputShape")){
        error_msg += "\nInputShape(w=" + std::to_string(IMAGE_WIDTH) +", h=" + std::to_string(IMAGE_HEIGHT) + ")\n";
        ailiaDestroyDetector(detector);
        ailiaDestroy(ailia);
        return;
    }
  }
}

AiliaYoloX::~AiliaYoloX(){
  ailiaDestroyDetector(detector);
  ailiaDestroy(ailia);
}

std::string AiliaYoloX::GetEnvironmentName(){
  AILIAEnvironment *env = nullptr;
  int status = ailiaGetSelectedEnvironment(ailia, &env, AILIA_ENVIRONMENT_VERSION);
  if(CheckError(status, "ailiaGetSelectedEnvironment")){
    return "";
  }
  return std::string(env->name);
}

std::vector<std::string> AiliaYoloX::GetAllEnvironmentNames(){
  std::vector<std::string> env_names;
  unsigned int env_count;
  int status;
	status = ailiaGetEnvironmentCount(&env_count);
  if(CheckError(status, "ailiaGetEnvironmentCount")){
    return env_names;
	}
  for(unsigned int i = 0; i < env_count; i++){
    AILIAEnvironment *env;
    ailiaGetEnvironment(&env, i, AILIA_ENVIRONMENT_VERSION);
    env_names.push_back(std::string(env->name));
  }
  return env_names;
}

std::string AiliaYoloX::GetAiliaVersion(){
  return std::string(ailiaGetVersion());
}

void AiliaYoloX::Preprocess(const cv::Mat& image, cv::Mat& out_image, float& out_scale)
{
  out_scale = std::min(float(IMAGE_WIDTH) / image.cols, float(IMAGE_HEIGHT) / image.rows);
  const auto scale_size = cv::Size(image.cols * out_scale, image.rows * out_scale);
  cv::resize(image, out_image, scale_size, 0, 0, cv::INTER_CUBIC);
  const auto bottom = IMAGE_HEIGHT - out_image.rows;
  const auto right = IMAGE_WIDTH - out_image.cols;
  cv::copyMakeBorder(
    out_image, out_image, 0, bottom, 0, right,
    cv::BORDER_CONSTANT, {0, 0, 0});
}

bool AiliaYoloX::doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  int status;
  objects.clear();
  for(auto& inimg : images){
    float scale = 1;
    cv::Mat img;

    Preprocess(inimg, img, scale);

    status = ailiaDetectorCompute(detector, img.data,
                                  img.cols * 3, img.cols, img.rows,
                                  AILIA_IMAGE_FORMAT_BGR, THRESHOLD, IOU);

    if(CheckError(status, "ailiaDetectorCompute")){
        return false;
    }

    uint obj_count;
    status = ailiaDetectorGetObjectCount(detector, &obj_count);
    if(CheckError(status, "ailiaDetectorGetObjectCount")){
        return false;
    }

    ObjectArray object_array(obj_count);
    for (uint j = 0; j < obj_count; ++j) {
      AILIADetectorObject obj;
      status = ailiaDetectorGetObject(detector, &obj, j, AILIA_DETECTOR_OBJECT_VERSION);
      if(CheckError(status, "ailiaDetectorGetObject")){
          return false;
      }

      Object object{};
      object.x_offset = std::clamp(0, int32_t(IMAGE_WIDTH * obj.x / scale), inimg.cols);
      object.y_offset = std::clamp(0, int32_t(IMAGE_HEIGHT * obj.y / scale), inimg.rows);
      object.width = int32_t(std::max(0.0F, IMAGE_WIDTH * obj.w / scale));
      object.height = int32_t(std::max(0.0F, IMAGE_HEIGHT * obj.h / scale));
      object.score = obj.prob;
      object.type = obj.category;
      object_array.emplace_back(object);
    }
    objects.emplace_back(object_array);
  }

  return true;
}

bool AiliaYoloX::CheckError(const int status, const std::string& funcName){
  if(status != AILIA_STATUS_SUCCESS){
    error_msg = funcName + " failed. " + ailiaGetStatusString(status) + "(" + std::to_string(status) + ")\n";
    error_msg += ailiaGetErrorDetail(ailia);
    error_ = true;
    return true;
  }
  return false;
}

bool AiliaYoloX::HasError(){
  return error_;
}

std::string AiliaYoloX::GetErrorDescription(){
  return error_msg;
}

}  // namespace tensorrt_yolox
