cmake_minimum_required(VERSION 3.5)
project(ailia_yolox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ailia_yolox.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/lib/libailia.so
  ${CMAKE_SOURCE_DIR}/lib/libailia_cuda-8.so
)
target_include_directories(${PROJECT_NAME}
  PUBLIC include/ailia
)
ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/ailia_yolox_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "ailia_yolox::AiliaYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
