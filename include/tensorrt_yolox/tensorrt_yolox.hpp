// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
#define TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <tensorrt_common/tensorrt_common.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tensorrt_yolox
{
using cuda_utils::CudaUniquePtr;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

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

class TrtYoloX
{
public:
  TrtYoloX(
    const std::string & model_path, const std::string & precision,
    const std::string & cache_dir = "",
    const tensorrt_common::BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1 << 30));

  bool doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects);

private:
  void preprocess(const std::vector<cv::Mat> & images);
  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  CudaUniquePtr<int32_t[]> out_num_detections_d_;
  CudaUniquePtr<float[]> out_boxes_d_;
  CudaUniquePtr<float[]> out_scores_d_;
  CudaUniquePtr<int32_t[]> out_classes_d_;

  StreamUniquePtr stream_{makeCudaStream()};

  int32_t max_detections_;
  std::vector<float> scales_;
};

}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
