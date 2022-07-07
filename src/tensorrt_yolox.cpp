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

#include <tensorrt_yolox/tensorrt_yolox.hpp>

#include <NvInferRuntime.h>

#include <efficientNMSPlugin.h>
#include <scatterPlugin.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace tensorrt_yolox
{
TrtYoloX::TrtYoloX(
  const std::string & model_path, const std::string & precision,
  [[maybe_unused]] const std::string & cache_dir,
  const tensorrt_common::BatchConfig & batch_config, const size_t max_workspace_size)
{
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
    model_path, precision, nullptr, batch_config, max_workspace_size);
  auto scatter_nd_instance = nvinfer1::plugin::ScatterNDPluginCreator{};
  auto efficient_nms_instance = nvinfer1::plugin::EfficientNMSPluginCreator{};
  getPluginRegistry()->registerCreator(scatter_nd_instance, "");
  getPluginRegistry()->registerCreator(efficient_nms_instance, "");
  trt_common_->setup();

  if (!trt_common_->isInitialized()) {
    return;
  }

  // GPU memory allocation
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const auto input_size = std::accumulate(
    input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());
  const auto out_scores_dims = trt_common_->getBindingDimensions(3);
  max_detections_ = out_scores_dims.d[1];
  input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
  out_num_detections_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2]);
  out_boxes_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_ * 4);
  out_scores_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_);
  out_classes_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2] * max_detections_);
}

void TrtYoloX::preprocess(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  scales_.clear();
  for (const auto & image : images) {
    cv::Mat dst_image;
    const float scale = std::min(input_width / image.cols, input_height / image.rows);
    scales_.emplace_back(scale);
    const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
    cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
    const auto bottom = input_height - dst_image.rows;
    const auto right = input_width - dst_image.cols;
    copyMakeBorder(
      dst_image, dst_image, 0, bottom, 0, right,
      cv::BORDER_CONSTANT, {114, 114, 114});
    dst_images.emplace_back(dst_image);
  }
  const auto chw_images = cv::dnn::blobFromImages(
    dst_images, 1.0, cv::Size(), cv::Scalar(), false, false, CV_32F);

  const auto data_length = chw_images.total();
  input_h_.reserve(data_length);
  const auto flat = chw_images.reshape(1, data_length);
  input_h_ = chw_images.isContinuous() ? flat : flat.clone();
}

bool TrtYoloX::doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }

  preprocess(images);

  CHECK_CUDA_ERROR(
    cudaMemcpy(
      input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice)
  );
  std::vector<void *> buffers = {
    input_d_.get(), out_num_detections_d_.get(), out_boxes_d_.get(),
    out_scores_d_.get(), out_classes_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  const auto batch_size = images.size();
  auto out_num_detections = std::make_unique<int32_t[]>(batch_size);
  auto out_boxes = std::make_unique<float[]>(4 * batch_size * max_detections_);
  auto out_scores = std::make_unique<float[]>(batch_size * max_detections_);
  auto out_classes = std::make_unique<float[]>(batch_size * max_detections_);

  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(
      out_num_detections.get(), out_num_detections_d_.get(),
      sizeof(int32_t) * batch_size, cudaMemcpyDeviceToHost,
      *stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(
      out_boxes.get(), out_boxes_d_.get(),
      sizeof(float) * 4 * batch_size * max_detections_, cudaMemcpyDeviceToHost,
      *stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(
      out_scores.get(), out_scores_d_.get(),
      sizeof(float) * batch_size * max_detections_, cudaMemcpyDeviceToHost,
      *stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(
      out_classes.get(), out_classes_d_.get(),
      sizeof(int32_t) * batch_size * max_detections_, cudaMemcpyDeviceToHost,
      *stream_));
  cudaStreamSynchronize(*stream_);
  objects.clear();
  for (size_t i = 0; i < batch_size; ++i) {
    const size_t num_detection = static_cast<size_t>(out_num_detections[i]);
    ObjectArray object_array(num_detection);
    for (size_t j = 0; j < num_detection; ++j) {
      Object object{};
      const auto x1 = out_boxes[i * max_detections_ * 4 + j * 4] / scales_[i];
      const auto y1 = out_boxes[i * max_detections_ * 4 + j * 4 + 1] / scales_[i];
      const auto x2 = out_boxes[i * max_detections_ * 4 + j * 4 + 2] / scales_[i];
      const auto y2 = out_boxes[i * max_detections_ * 4 + j * 4 + 3] / scales_[i];
      object.x_offset = std::clamp(0, static_cast<int32_t>(x1), images[i].cols);
      object.y_offset = std::clamp(0, static_cast<int32_t>(y1), images[i].rows);
      object.width = static_cast<int32_t>(std::max(0.0F, x2 - x1));
      object.height = static_cast<int32_t>(std::max(0.0F, y2 - y1));
      object.score = out_scores[i * max_detections_ + j];
      object.type = out_classes[i * max_detections_ + j];
      object_array.emplace_back(object);
    }
    objects.emplace_back(object_array);
  }
  return true;
}

}  // namespace tensorrt_yolox
