#include "maixsense_a075v/tof_camera_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace maixsense_a075v
{
namespace
{

constexpr size_t kPixelCount =
  static_cast<size_t>(kDepthWidth) * kDepthHeight;
constexpr auto kRetryDelay = std::chrono::milliseconds(200);

template<typename T>
void write_value(uint8_t * destination, const T & value)
{
  std::memcpy(destination, &value, sizeof(T));
}

sensor_msgs::msg::PointField make_field(
  const char * name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

bool acquisition_equal(
  const AcquisitionConfig & lhs, const AcquisitionConfig & rhs)
{
  return lhs.trigger_mode == rhs.trigger_mode &&
         lhs.depth_mode == rhs.depth_mode &&
         lhs.depth_shift == rhs.depth_shift &&
         lhs.infrared_mode == rhs.infrared_mode &&
         lhs.status_mode == rhs.status_mode &&
         lhs.status_mask == rhs.status_mask &&
         lhs.rgb_mode == rhs.rgb_mode &&
         lhs.rgb_resolution == rhs.rgb_resolution &&
         lhs.exposure_time == rhs.exposure_time;
}

}  // namespace

TofCameraNode::TofCameraNode(const rclcpp::NodeOptions & options)
: LifecycleNode("maixsense_a075v", options),
  param_listener_(
    std::make_shared<ParamListener>(get_node_parameters_interface())),
  params_(param_listener_->get_params())
{
}

TofCameraNode::~TofCameraNode()
{
  stop_acquisition();
}

TofCameraNode::CallbackReturn TofCameraNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  if (params_.spatial_kernel_size % 2 == 0) {
    RCLCPP_ERROR(get_logger(), "spatial_kernel_size must be odd");
    return CallbackReturn::FAILURE;
  }
  if (params_.spatial_filter_type != "gaussian" &&
    params_.spatial_filter_type != "bilateral")
  {
    RCLCPP_ERROR(
      get_logger(), "spatial_filter_type must be gaussian or bilateral");
    return CallbackReturn::FAILURE;
  }

  device_ = std::make_unique<Device>(
    params_.host, static_cast<uint16_t>(params_.port),
    std::chrono::milliseconds(
      static_cast<int64_t>(params_.request_timeout * 1000.0)));
  const auto configure_timeout = std::chrono::milliseconds(
    static_cast<int64_t>(params_.configure_timeout * 1000.0));
  if (!device_->wait_until_ready(configure_timeout)) {
    RCLCPP_ERROR(
      get_logger(), "Camera did not become ready: %s",
      device_->error().c_str());
    release_resources();
    return CallbackReturn::FAILURE;
  }
  if (!device_->load_calibration()) {
    RCLCPP_ERROR(
      get_logger(), "Could not load camera calibration: %s",
      device_->error().c_str());
    release_resources();
    return CallbackReturn::FAILURE;
  }
  device_->set_pipeline_config(pipeline_config());

  const auto sensor_qos = rclcpp::SensorDataQoS();
  point_cloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("cloud", sensor_qos);
  color_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("rgb/image", sensor_qos);
  depth_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("tof/depth", sensor_qos);
  intensity_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("tof/intensity", sensor_qos);
  status_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("tof/status", sensor_qos);
  tof_camera_info_publisher_ =
    create_publisher<sensor_msgs::msg::CameraInfo>(
    "tof/camera_info", sensor_qos);
  rgb_camera_info_publisher_ =
    create_publisher<sensor_msgs::msg::CameraInfo>(
    "rgb/camera_info", sensor_qos);

  initialize_calibration_messages();
  if (params_.publish_static_transforms) {
    static_transform_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    broadcast_static_transforms();
  }
  RCLCPP_INFO(
    get_logger(), "Configured camera at %s:%ld", params_.host.c_str(),
    params_.port);
  return CallbackReturn::SUCCESS;
}

TofCameraNode::CallbackReturn TofCameraNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  const auto parent_result = LifecycleNode::on_activate(state);
  if (parent_result != CallbackReturn::SUCCESS) {
    return parent_result;
  }
  if (!device_->set_acquisition_config(acquisition_config(1))) {
    LifecycleNode::on_deactivate(state);
    RCLCPP_ERROR(
      get_logger(), "Could not start camera: %s", device_->error().c_str());
    return CallbackReturn::FAILURE;
  }

  acquiring_ = true;
  acquisition_thread_ = std::thread(&TofCameraNode::acquisition_loop, this);
  RCLCPP_INFO(get_logger(), "Camera acquisition active");
  return CallbackReturn::SUCCESS;
}

TofCameraNode::CallbackReturn TofCameraNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  stop_acquisition();
  if (device_ && !device_->set_acquisition_config(acquisition_config(0))) {
    RCLCPP_WARN(
      get_logger(), "Could not stop camera trigger: %s",
      device_->error().c_str());
  }
  const auto result = LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "Camera acquisition inactive");
  return result;
}

TofCameraNode::CallbackReturn TofCameraNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_acquisition();
  release_resources();
  return CallbackReturn::SUCCESS;
}

TofCameraNode::CallbackReturn TofCameraNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_acquisition();
  release_resources();
  return CallbackReturn::SUCCESS;
}

TofCameraNode::CallbackReturn TofCameraNode::on_error(
  const rclcpp_lifecycle::State &)
{
  stop_acquisition();
  release_resources();
  return CallbackReturn::SUCCESS;
}

AcquisitionConfig TofCameraNode::acquisition_config(
  uint8_t trigger_mode) const
{
  AcquisitionConfig config;
  config.trigger_mode = trigger_mode;
  config.depth_mode = static_cast<uint8_t>(params_.depth_mode);
  config.depth_shift = static_cast<uint8_t>(params_.depth_shift);
  config.infrared_mode = static_cast<uint8_t>(params_.infrared_mode);
  config.status_mode = static_cast<uint8_t>(params_.status_mode);
  config.status_mask = static_cast<uint8_t>(params_.status_mask);
  config.rgb_mode = static_cast<uint8_t>(params_.rgb_mode);
  config.rgb_resolution = static_cast<uint8_t>(params_.rgb_resolution);
  config.exposure_time = static_cast<int32_t>(params_.exposure_time);
  return config;
}

PipelineConfig TofCameraNode::pipeline_config() const
{
  PipelineConfig config;
  config.temporal_filter = params_.temporal_filter;
  config.temporal_alpha = params_.temporal_alpha;
  config.spatial_filter = params_.spatial_filter;
  config.spatial_filter_type = params_.spatial_filter_type == "bilateral" ?
    SpatialFilterType::kBilateral : SpatialFilterType::kGaussian;
  config.spatial_kernel_size = static_cast<int>(params_.spatial_kernel_size);
  config.flying_point_filter = params_.flying_point_filter;
  config.flying_point_threshold = params_.flying_point_threshold;
  config.keep_normal = params_.keep_normal;
  config.keep_overexposed = params_.keep_overexposed;
  config.keep_underexposed = params_.keep_underexposed;
  config.keep_bad = params_.keep_bad;
  return config;
}

void TofCameraNode::initialize_calibration_messages()
{
  const Calibration & calibration = device_->calibration();
  const auto initialize =
    [](const CameraCalibration & input, const std::string & frame_id,
      bool rectified, sensor_msgs::msg::CameraInfo & output)
    {
      output.header.frame_id = frame_id;
      output.width = input.width;
      output.height = input.height;
      output.distortion_model = "plumb_bob";
      if (rectified) {
        output.d.assign(input.distortion.size(), 0.0);
      } else {
        output.d.assign(input.distortion.begin(), input.distortion.end());
      }
      output.k = input.camera_matrix;
      output.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      output.p = {
        input.camera_matrix[0], input.camera_matrix[1],
        input.camera_matrix[2], 0.0,
        input.camera_matrix[3], input.camera_matrix[4],
        input.camera_matrix[5], 0.0,
        input.camera_matrix[6], input.camera_matrix[7],
        input.camera_matrix[8], 0.0};
    };
  initialize(
    calibration.tof, params_.tof_optical_frame, true, tof_camera_info_);
  initialize(
    calibration.rgb, params_.rgb_optical_frame, false, rgb_camera_info_);
}

void TofCameraNode::broadcast_static_transforms()
{
  // Both sensors sit on the same board and look along the body x axis, so the
  // two optical frames share the REP-103 optical rotation: z forward, x right,
  // y down. Their positions come from the STEP model, not from the factory
  // RGB-ToF calibration, whose translation is a projection artefact rather
  // than a mechanical baseline.
  tf2::Quaternion optical_rotation;
  optical_rotation.setRPY(-M_PI_2, 0.0, -M_PI_2);

  const auto make_transform =
    [this, &optical_rotation](
    const SensorMount & mount, const std::string & child_frame_id)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = params_.body_frame;
      transform.child_frame_id = child_frame_id;
      transform.transform.translation.x = mount.x;
      transform.transform.translation.y = mount.y;
      transform.transform.translation.z = mount.z;
      transform.transform.rotation.x = optical_rotation.x();
      transform.transform.rotation.y = optical_rotation.y();
      transform.transform.rotation.z = optical_rotation.z();
      transform.transform.rotation.w = optical_rotation.w();
      return transform;
    };

  const std::vector<geometry_msgs::msg::TransformStamped> transforms{
    make_transform(kTofSensorMount, params_.tof_optical_frame),
    make_transform(kRgbSensorMount, params_.rgb_optical_frame)};
  static_transform_broadcaster_->sendTransform(transforms);
}

void TofCameraNode::acquisition_loop()
{
  auto last_acquisition_config = acquisition_config(1);
  while (acquiring_) {
    const auto started = std::chrono::steady_clock::now();
    if (!update_runtime_config()) {
      std::this_thread::sleep_for(kRetryDelay);
      continue;
    }

    const auto next_acquisition_config = acquisition_config(1);
    if (!acquisition_equal(
        last_acquisition_config, next_acquisition_config))
    {
      if (!device_->set_acquisition_config(next_acquisition_config)) {
        RCLCPP_WARN(
          get_logger(), "Could not apply camera parameters: %s",
          device_->error().c_str());
        std::this_thread::sleep_for(kRetryDelay);
        continue;
      }
      last_acquisition_config = next_acquisition_config;
    }

    DecodedFrame frame;
    if (!device_->read_frame(frame)) {
      RCLCPP_WARN(
        get_logger(), "Could not read frame: %s", device_->error().c_str());
      std::this_thread::sleep_for(kRetryDelay);
      continue;
    }
    publish_frame(frame);

    const auto period = std::chrono::duration<double>(params_.frame_period);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

void TofCameraNode::publish_frame(const DecodedFrame & frame)
{
  if (frame.depth_mm.size() != kPixelCount ||
    frame.intensity.size() != kPixelCount ||
    frame.status.size() != kPixelCount)
  {
    RCLCPP_WARN(get_logger(), "Discarded decoded frame with invalid dimensions");
    return;
  }

  std_msgs::msg::Header tof_header;
  tof_header.stamp = now();
  tof_header.frame_id = params_.tof_optical_frame;
  tof_camera_info_.header = tof_header;
  tof_camera_info_publisher_->publish(tof_camera_info_);

  cv::Mat depth(
    kDepthHeight, kDepthWidth, CV_16UC1,
    const_cast<uint16_t *>(frame.depth_mm.data()));
  cv::Mat intensity(
    kDepthHeight, kDepthWidth, CV_16UC1,
    const_cast<uint16_t *>(frame.intensity.data()));
  cv::Mat status(
    kDepthHeight, kDepthWidth, CV_16UC1,
    const_cast<uint16_t *>(frame.status.data()));
  depth_publisher_->publish(
    *cv_bridge::CvImage(tof_header, "mono16", depth).toImageMsg());
  intensity_publisher_->publish(
    *cv_bridge::CvImage(tof_header, "mono16", intensity).toImageMsg());
  status_publisher_->publish(
    *cv_bridge::CvImage(tof_header, "mono16", status).toImageMsg());

  if (!frame.rgb_bgr.empty()) {
    std_msgs::msg::Header rgb_header;
    rgb_header.stamp = tof_header.stamp;
    rgb_header.frame_id = params_.rgb_optical_frame;
    rgb_camera_info_.header = rgb_header;
    const auto & base = device_->calibration().rgb;
    const double scale_x =
      static_cast<double>(frame.rgb_width) / base.width;
    const double scale_y =
      static_cast<double>(frame.rgb_height) / base.height;
    rgb_camera_info_.width = frame.rgb_width;
    rgb_camera_info_.height = frame.rgb_height;
    rgb_camera_info_.k = base.camera_matrix;
    rgb_camera_info_.k[0] *= scale_x;
    rgb_camera_info_.k[1] *= scale_x;
    rgb_camera_info_.k[2] *= scale_x;
    rgb_camera_info_.k[4] *= scale_y;
    rgb_camera_info_.k[5] *= scale_y;
    rgb_camera_info_.p[0] = rgb_camera_info_.k[0];
    rgb_camera_info_.p[1] = rgb_camera_info_.k[1];
    rgb_camera_info_.p[2] = rgb_camera_info_.k[2];
    rgb_camera_info_.p[5] = rgb_camera_info_.k[4];
    rgb_camera_info_.p[6] = rgb_camera_info_.k[5];
    rgb_camera_info_publisher_->publish(rgb_camera_info_);
    cv::Mat rgb(
      static_cast<int>(frame.rgb_height), static_cast<int>(frame.rgb_width),
      CV_8UC3, const_cast<uint8_t *>(frame.rgb_bgr.data()));
    color_publisher_->publish(
      *cv_bridge::CvImage(rgb_header, "bgr8", rgb).toImageMsg());
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = tof_header;
  cloud.height = kDepthHeight;
  cloud.width = kDepthWidth;
  cloud.is_bigendian = false;
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = false;
  cloud.fields = {
    make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    make_field("rgb", 12, sensor_msgs::msg::PointField::UINT32),
    make_field("intensity", 16, sensor_msgs::msg::PointField::FLOAT32)};
  cloud.data.resize(kPixelCount * cloud.point_step);

  const auto & matrix = device_->calibration().tof.camera_matrix;
  const float fx = static_cast<float>(matrix[0]);
  const float fy = static_cast<float>(matrix[4]);
  const float cx = static_cast<float>(matrix[2]);
  const float cy = static_cast<float>(matrix[5]);
  uint8_t * point = cloud.data.data();
  for (uint32_t row = 0; row < kDepthHeight; ++row) {
    for (uint32_t column = 0; column < kDepthWidth;
      ++column, point += cloud.point_step)
    {
      const size_t index =
        static_cast<size_t>(row) * kDepthWidth + column;
      const float distance = frame.depth_mm[index] / 1000.0F;
      float x = distance * (static_cast<float>(column) - cx) / fx;
      float y = distance * (static_cast<float>(row) - cy) / fy;
      float z = distance;
      if (frame.depth_mm[index] == 0) {
        x = y = z = std::numeric_limits<float>::quiet_NaN();
      }
      uint32_t rgb = 0;
      if (frame.mapped_rgba.size() == kPixelCount) {
        const uint32_t rgba = frame.mapped_rgba[index];
        rgb = ((rgba & 0x000000FFU) << 16U) |
          (rgba & 0x0000FF00U) |
          ((rgba & 0x00FF0000U) >> 16U);
      }
      const float signal = static_cast<float>(frame.intensity[index]);
      write_value(point, x);
      write_value(point + 4, y);
      write_value(point + 8, z);
      write_value(point + 12, rgb);
      write_value(point + 16, signal);
    }
  }
  point_cloud_publisher_->publish(cloud);
}

bool TofCameraNode::update_runtime_config()
{
  if (!param_listener_->try_update_params(params_)) {
    return true;
  }
  if (params_.spatial_kernel_size % 2 == 0 ||
    (params_.spatial_filter_type != "gaussian" &&
    params_.spatial_filter_type != "bilateral"))
  {
    RCLCPP_ERROR(
      get_logger(), "Rejected invalid runtime filter parameters");
    return false;
  }
  device_->set_pipeline_config(pipeline_config());
  return true;
}

void TofCameraNode::stop_acquisition()
{
  acquiring_ = false;
  if (acquisition_thread_.joinable()) {
    acquisition_thread_.join();
  }
}

void TofCameraNode::release_resources()
{
  point_cloud_publisher_.reset();
  color_publisher_.reset();
  depth_publisher_.reset();
  intensity_publisher_.reset();
  status_publisher_.reset();
  tof_camera_info_publisher_.reset();
  rgb_camera_info_publisher_.reset();
  static_transform_broadcaster_.reset();
  device_.reset();
}

}  // namespace maixsense_a075v

RCLCPP_COMPONENTS_REGISTER_NODE(maixsense_a075v::TofCameraNode)
