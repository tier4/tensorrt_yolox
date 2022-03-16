cmake_minimum_required(VERSION 3.5)
project(tensorrt_yolox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-deprecated-declarations -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

set(TENSORRT_BASE_URL https://raw.githubusercontent.com/NVIDIA/TensorRT/8.0.1)

set(TENSORRT_ADDITIONAL_PLUGIN_SRCS
  plugin/common/bboxUtils.h
  plugin/common/checkMacrosPlugin.h
  plugin/common/kernels/common.cu
  plugin/common/kernels/kernel.h
  plugin/common/plugin.h
  plugin/efficientNMSPlugin/efficientNMSInference.cu
  plugin/efficientNMSPlugin/efficientNMSInference.cuh
  plugin/efficientNMSPlugin/efficientNMSInference.h
  plugin/efficientNMSPlugin/efficientNMSParameters.h
  plugin/efficientNMSPlugin/efficientNMSPlugin.cpp
  plugin/efficientNMSPlugin/efficientNMSPlugin.h
  plugin/scatterPlugin/scatterLayer.cu
  plugin/scatterPlugin/scatterPlugin.cpp
  plugin/scatterPlugin/scatterPlugin.h
  samples/common/half.h
)

foreach(SRC IN LISTS TENSORRT_ADDITIONAL_PLUGIN_SRCS)
  file(DOWNLOAD
    ${TENSORRT_BASE_URL}/${SRC}
    ${CMAKE_SOURCE_DIR}/src/${SRC}
    SHOW_PROGRESS
    TIMEOUT 60s
  )
  list(APPEND PLUGIN_SRCS src/${SRC})
endforeach()

include(ExternalProject)
ExternalProject_Add(
  cub
  GIT_REPOSITORY https://github.com/NVIDIA/cub
  GIT_TAG 1.9.10
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/cub
  UPDATE_COMMAND    ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
)

cuda_add_library(tensorrt_additional_plugins SHARED
  ${PLUGIN_SRCS}
  ${TENSORRT_LIBRARIES}
)

add_dependencies(tensorrt_additional_plugins cub)

target_include_directories(tensorrt_additional_plugins PUBLIC
  src/cub
  src/plugin/common
  src/plugin/common/kernels
  src/plugin/efficientNMSPlugin
  src/plugin/scatterPlugin
  src/samples/common
  ${TENSORRT_INCLUDE_DIRS}
)

list(APPEND ${PROJECT_NAME}_LIBRARIES tensorrt_additional_plugins)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

target_link_libraries(${PROJECT_NAME}
  tensorrt_additional_plugins
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inferece_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inferece_node
  OpenCV
)

target_link_libraries(yolox_single_image_inferece_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inferece_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inferece_node
  PLUGIN "tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inferece
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  data
  launch
)
