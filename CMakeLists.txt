cmake_minimum_required(VERSION 3.16)
project(maixsense_a075v)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

generate_parameter_library(
  maixsense_a075v_parameters
  src/maixsense_a075v_parameters.yaml
)

add_library(maixsense_a075v SHARED
  src/device.cpp
  src/frame_pipeline.cpp
)
target_include_directories(maixsense_a075v PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(maixsense_a075v PUBLIC
  PRIVATE
  CURL::libcurl
  ${OpenCV_LIBS}
  Threads::Threads
)

add_library(maixsense_a075v_node SHARED
  src/tof_camera_node.cpp
)
target_include_directories(maixsense_a075v_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(maixsense_a075v_node PUBLIC
  maixsense_a075v
  maixsense_a075v_parameters
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  cv_bridge::cv_bridge
  geometry_msgs::geometry_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  PRIVATE
  Threads::Threads
)

rclcpp_components_register_node(maixsense_a075v_node
  PLUGIN "maixsense_a075v::TofCameraNode"
  EXECUTABLE tof_camera_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  TARGETS maixsense_a075v
  EXPORT maixsense_a075v_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS maixsense_a075v_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frame_pipeline test/test_frame_pipeline.cpp)
  target_include_directories(test_frame_pipeline PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_frame_pipeline maixsense_a075v ${OpenCV_LIBS})
endif()

ament_export_targets(maixsense_a075v_targets HAS_LIBRARY_TARGET)
ament_package()
