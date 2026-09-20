#include "frame_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace maixsense_a075v
{
namespace
{

constexpr size_t kPixelCount =
  static_cast<size_t>(kDepthWidth) * kDepthHeight;
constexpr size_t kDepthLutSize = 65536;

#pragma pack(push, 1)
struct WireConfig
{
  uint8_t trigger_mode;
  uint8_t depth_mode;
  uint8_t depth_shift;
  uint8_t infrared_mode;
  uint8_t status_mode;
  uint8_t status_mask;
  uint8_t rgb_mode;
  uint8_t rgb_resolution;
  int32_t exposure_time;
};

struct WireFrameHeader
{
  uint64_t sequence;
  uint64_t stamp_ms;
  WireConfig config;
  int32_t non_rgb_size;
  int32_t rgb_size;
};

struct LensCoefficients
{
  uint8_t calibration_mode;
  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float k3;
  float p1;
  float p2;
  float skew;
};

struct ModuleInformation
{
  char sensor_part_number[9];
  char module_vendor[2];
  char module_type[2];
  char module_serial[16];
  char vcsel_id[4];
  char bin_version[3];
  uint8_t calibration_algorithm_version[2];
  uint8_t firmware_version[2];
};

struct WireModuleInfo
{
  ModuleInformation module;
  LensCoefficients lens;
};
#pragma pack(pop)

static_assert(sizeof(WireConfig) == 12);
static_assert(sizeof(WireFrameHeader) == 36);
static_assert(sizeof(WireModuleInfo) == 81);

size_t depth_size(const WireConfig & config)
{
  return config.depth_mode == 1 ? kPixelCount : kPixelCount * sizeof(uint16_t);
}

size_t infrared_size(const WireConfig & config)
{
  return config.infrared_mode == 1 ? kPixelCount :
         kPixelCount * sizeof(uint16_t);
}

size_t status_size(const WireConfig & config)
{
  switch (config.status_mode) {
    case 1:
      return kPixelCount / 4;
    case 2:
      return kPixelCount;
    case 3:
      return kPixelCount / 8;
    default:
      return kPixelCount * sizeof(uint16_t);
  }
}

template<size_t N>
bool read_json_array(
  const cv::FileStorage & storage, const char * name,
  std::array<double, N> & output)
{
  const cv::FileNode node = storage[name];
  if (!node.isSeq() || node.size() != N) {
    return false;
  }
  for (size_t index = 0; index < N; ++index) {
    output[index] = static_cast<double>(node[static_cast<int>(index)]);
  }
  return true;
}

void decode_16_bit(
  const uint8_t * input, size_t count, std::vector<uint16_t> & output)
{
  output.resize(count);
  std::memcpy(output.data(), input, count * sizeof(uint16_t));
}

}  // namespace

FramePipeline::FramePipeline()
{
  calibration_.tof.width = kDepthWidth;
  calibration_.tof.height = kDepthHeight;
  calibration_.rgb.width = kRgbWidth;
  calibration_.rgb.height = kRgbHeight;
}

bool FramePipeline::set_module_info(
  const std::vector<uint8_t> & data, std::string & error)
{
  if (data.size() != sizeof(WireModuleInfo)) {
    error = "module information has " + std::to_string(data.size()) +
      " bytes; expected " + std::to_string(sizeof(WireModuleInfo));
    return false;
  }

  WireModuleInfo info{};
  std::memcpy(&info, data.data(), sizeof(info));
  const auto & lens = info.lens;
  if (!std::isfinite(lens.fx) || !std::isfinite(lens.fy) ||
    lens.fx <= 0.0F || lens.fy <= 0.0F)
  {
    error = "module information contains invalid ToF focal lengths";
    return false;
  }

  calibration_.tof.camera_matrix = {
    lens.fx, lens.skew, lens.cx,
    0.0, lens.fy, lens.cy,
    0.0, 0.0, 1.0};
  calibration_.tof.distortion = {
    lens.k1, lens.k2, lens.p1, lens.p2, lens.k3};

  cv::Mat camera_matrix(3, 3, CV_64F, calibration_.tof.camera_matrix.data());
  cv::Mat distortion(1, 5, CV_64F, calibration_.tof.distortion.data());
  cv::initUndistortRectifyMap(
    camera_matrix, distortion, cv::Mat(), camera_matrix,
    cv::Size(kDepthWidth, kDepthHeight), CV_32FC1, rectify_map_x_,
    rectify_map_y_);
  module_info_ready_ = true;
  return true;
}

bool FramePipeline::set_depth_lut(
  const std::vector<uint8_t> & data, std::string & error)
{
  if (data.size() < kDepthLutSize) {
    error = "depth LUT has " + std::to_string(data.size()) +
      " bytes; expected at least " + std::to_string(kDepthLutSize);
    return false;
  }

  std::array<double, 256> sums{};
  std::array<uint32_t, 256> counts{};
  for (size_t index = 0; index < kDepthLutSize; ++index) {
    const uint8_t code = data[index];
    sums[code] += static_cast<double>(index);
    ++counts[code];
  }
  for (size_t code = 0; code < depth_lut_.size(); ++code) {
    depth_lut_[code] = counts[code] == 0 ? 0 :
      static_cast<uint16_t>(sums[code] / counts[code]);
  }
  lut_ready_ = true;
  return true;
}

bool FramePipeline::set_rgb_calibration(
  const std::vector<uint8_t> & json, std::string & error)
{
  const std::string text(json.begin(), json.end());
  cv::FileStorage storage(
    text, cv::FileStorage::READ | cv::FileStorage::MEMORY |
    cv::FileStorage::FORMAT_JSON);
  if (!storage.isOpened()) {
    error = "RGB calibration is not valid JSON";
    return false;
  }

  std::array<double, 9> rotation{};
  std::array<double, 3> translation_mm{};
  if (!read_json_array(storage, "R_Matrix_data", rotation) ||
    !read_json_array(storage, "T_Vec_data", translation_mm) ||
    !read_json_array(
      storage, "Camera_Matrix_data", calibration_.rgb.camera_matrix) ||
    !read_json_array(
      storage, "Distortion_Parm_data", calibration_.rgb.distortion))
  {
    error = "RGB calibration JSON is incomplete";
    return false;
  }

  // Sipeed's projection code converts points from ToF (camera 2) to RGB
  // (camera 1) with R transpose and negative T. Preserve that convention.
  cv::Mat source_rotation(3, 3, CV_64F, rotation.data());
  cv::transpose(source_rotation, rotation_);
  translation_mm_ = cv::Mat(3, 1, CV_64F);
  for (size_t index = 0; index < translation_mm.size(); ++index) {
    translation_mm_.at<double>(static_cast<int>(index)) =
      -translation_mm[index];
    calibration_.tof_to_rgb_translation_m[index] =
      -translation_mm[index] / 1000.0;
  }
  std::copy(
    rotation_.begin<double>(), rotation_.end<double>(),
    calibration_.tof_to_rgb_rotation.begin());

  rgb_camera_matrix_ =
    cv::Mat(3, 3, CV_64F, calibration_.rgb.camera_matrix.data()).clone();
  rgb_distortion_ =
    cv::Mat(1, 5, CV_64F, calibration_.rgb.distortion.data()).clone();
  rgb_calibration_ready_ = true;
  return true;
}

void FramePipeline::set_config(const PipelineConfig & config)
{
  if (config.temporal_alpha != config_.temporal_alpha ||
    config.temporal_filter != config_.temporal_filter)
  {
    reset_temporal_filter();
  }
  config_ = config;
}

bool FramePipeline::decode(
  const std::vector<uint8_t> & payload, DecodedFrame & frame,
  std::string & error)
{
  if (!module_info_ready_ || !lut_ready_) {
    error = "calibration and depth LUT must be loaded before decoding";
    return false;
  }
  if (payload.size() < sizeof(WireFrameHeader)) {
    error = "frame header is truncated";
    return false;
  }

  WireFrameHeader header{};
  std::memcpy(&header, payload.data(), sizeof(header));
  if (header.rgb_size < 0 || header.non_rgb_size < 0) {
    error = "frame contains a negative payload size";
    return false;
  }

  const size_t depth_bytes = depth_size(header.config);
  const size_t infrared_bytes = infrared_size(header.config);
  const size_t status_bytes = status_size(header.config);
  const size_t non_rgb_bytes = depth_bytes + infrared_bytes + status_bytes;
  const size_t expected = sizeof(header) + non_rgb_bytes +
    static_cast<size_t>(header.rgb_size);
  if (header.non_rgb_size != static_cast<int32_t>(non_rgb_bytes) ||
    payload.size() < expected)
  {
    error = "frame payload sizes are inconsistent";
    return false;
  }

  const uint8_t * cursor = payload.data() + sizeof(header);
  std::vector<uint16_t> raw_depth;
  if (header.config.depth_mode == 1) {
    raw_depth.resize(kPixelCount);
    for (size_t index = 0; index < kPixelCount; ++index) {
      raw_depth[index] = header.config.depth_shift == 255 ?
        depth_lut_[cursor[index]] :
        static_cast<uint16_t>(cursor[index] << header.config.depth_shift);
    }
  } else {
    decode_16_bit(cursor, kPixelCount, raw_depth);
  }
  cursor += depth_bytes;

  std::vector<uint16_t> raw_intensity;
  if (header.config.infrared_mode == 1) {
    raw_intensity.resize(kPixelCount);
    for (size_t index = 0; index < kPixelCount; ++index) {
      raw_intensity[index] = static_cast<uint16_t>(cursor[index]) * 16U;
    }
  } else {
    decode_16_bit(cursor, kPixelCount, raw_intensity);
  }
  cursor += infrared_bytes;

  std::vector<uint16_t> raw_status(kPixelCount);
  switch (header.config.status_mode) {
    case 1:
      for (size_t index = 0; index < kPixelCount; ++index) {
        raw_status[index] =
          (cursor[index / 4] >> (2 * (index % 4))) & 0x03U;
      }
      break;
    case 2:
      for (size_t index = 0; index < kPixelCount; ++index) {
        raw_status[index] = cursor[index];
      }
      break;
    case 3:
      for (size_t index = 0; index < kPixelCount; ++index) {
        raw_status[index] = ((cursor[index / 8] >> (index % 8)) & 1U) ? 3 : 0;
      }
      break;
    default:
      decode_16_bit(cursor, kPixelCount, raw_status);
      break;
  }
  cursor += status_bytes;

  apply_filters(raw_depth);
  rectify(raw_depth, frame.depth_mm);
  rectify(raw_intensity, frame.intensity);
  rectify(raw_status, frame.status);

  for (size_t index = 0; index < frame.status.size(); ++index) {
    const uint16_t status = frame.status[index];
    const bool keep =
      (status == 0 && config_.keep_normal) ||
      (status == 1 && config_.keep_underexposed) ||
      (status == 2 && config_.keep_overexposed) ||
      (status == 3 && config_.keep_bad);
    if (!keep) {
      frame.depth_mm[index] = 0;
    }
  }

  frame.rgb_bgr.clear();
  frame.rgb_width = 0;
  frame.rgb_height = 0;
  if (header.config.rgb_mode == 1 && header.rgb_size > 0) {
    cv::Mat encoded(
      1, header.rgb_size, CV_8UC1, const_cast<uint8_t *>(cursor));
    cv::Mat color = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (color.empty()) {
      error = "JPEG payload could not be decoded";
      return false;
    }
    frame.rgb_width = color.cols;
    frame.rgb_height = color.rows;
    frame.rgb_bgr.assign(color.datastart, color.dataend);
  } else if (header.config.rgb_mode == 0 && header.rgb_size > 0) {
    const int width = header.config.rgb_resolution == 0 ? 800 : 1600;
    const int height = header.config.rgb_resolution == 0 ? 600 : 1200;
    const size_t yuv_size =
      static_cast<size_t>(width) * height * 3 / 2;
    if (static_cast<size_t>(header.rgb_size) != yuv_size) {
      error = "YUV payload has an unexpected size";
      return false;
    }
    cv::Mat yuv(
      height + height / 2, width, CV_8UC1,
      const_cast<uint8_t *>(cursor));
    cv::Mat color;
    cv::cvtColor(yuv, color, cv::COLOR_YUV2BGR_NV21);
    frame.rgb_width = color.cols;
    frame.rgb_height = color.rows;
    frame.rgb_bgr.assign(color.datastart, color.dataend);
  }

  frame.sequence = header.sequence;
  frame.device_stamp_ms = header.stamp_ms;
  map_rgb_to_tof(frame);
  return true;
}

const Calibration & FramePipeline::calibration() const
{
  return calibration_;
}

void FramePipeline::reset_temporal_filter()
{
  previous_depth_.clear();
}

void FramePipeline::apply_filters(std::vector<uint16_t> & depth)
{
  cv::Mat image(kDepthHeight, kDepthWidth, CV_16UC1, depth.data());

  if (config_.temporal_filter) {
    if (previous_depth_.size() == depth.size()) {
      const double alpha = std::clamp(config_.temporal_alpha, 0.0, 1.0);
      for (size_t index = 0; index < depth.size(); ++index) {
        depth[index] = static_cast<uint16_t>(
          alpha * depth[index] + (1.0 - alpha) * previous_depth_[index]);
      }
    }
    previous_depth_ = depth;
  }

  if (config_.spatial_filter) {
    int kernel = std::max(1, config_.spatial_kernel_size);
    if (kernel % 2 == 0) {
      ++kernel;
    }
    cv::Mat filtered;
    if (config_.spatial_filter_type == SpatialFilterType::kBilateral) {
      cv::Mat float_image;
      cv::Mat float_filtered;
      image.convertTo(float_image, CV_32F);
      cv::bilateralFilter(float_image, float_filtered, kernel, 50.0, 50.0);
      float_filtered.convertTo(filtered, CV_16U);
    } else {
      cv::GaussianBlur(image, filtered, cv::Size(kernel, kernel), 0.0);
    }
    std::memcpy(depth.data(), filtered.data, depth.size() * sizeof(uint16_t));
  }

  if (config_.flying_point_filter) {
    cv::Mat float_depth;
    image.convertTo(float_depth, CV_32F);
    cv::Mat dilated;
    cv::Mat eroded;
    cv::dilate(float_depth, dilated, cv::Mat());
    cv::erode(float_depth, eroded, cv::Mat());
    cv::Mat relative;
    cv::divide(dilated - eroded, float_depth, relative);
    const float threshold =
      static_cast<float>(config_.flying_point_threshold);
    for (int row = 0; row < image.rows; ++row) {
      for (int column = 0; column < image.cols; ++column) {
        const float value = relative.at<float>(row, column);
        if (!std::isfinite(value) || value >= threshold) {
          image.at<uint16_t>(row, column) = 0;
        }
      }
    }
  }
}

void FramePipeline::rectify(
  const std::vector<uint16_t> & input, std::vector<uint16_t> & output) const
{
  cv::Mat source(
    kDepthHeight, kDepthWidth, CV_16UC1,
    const_cast<uint16_t *>(input.data()));
  cv::Mat destination;
  cv::remap(
    source, destination, rectify_map_x_, rectify_map_y_, cv::INTER_NEAREST,
    cv::BORDER_CONSTANT, cv::Scalar(0));
  output.assign(
    destination.begin<uint16_t>(), destination.end<uint16_t>());
}

void FramePipeline::map_rgb_to_tof(DecodedFrame & frame) const
{
  frame.mapped_rgba.assign(kPixelCount, 0);
  if (!rgb_calibration_ready_ || frame.rgb_bgr.empty()) {
    return;
  }

  const double fx = calibration_.tof.camera_matrix[0];
  const double fy = calibration_.tof.camera_matrix[4];
  const double cx = calibration_.tof.camera_matrix[2];
  const double cy = calibration_.tof.camera_matrix[5];
  std::vector<cv::Point3f> points(kPixelCount);
  for (uint32_t row = 0; row < kDepthHeight; ++row) {
    for (uint32_t column = 0; column < kDepthWidth; ++column) {
      const size_t index = static_cast<size_t>(row) * kDepthWidth + column;
      const float z = static_cast<float>(frame.depth_mm[index]);
      points[index] = cv::Point3f(
        z * static_cast<float>((column - cx) / fx),
        z * static_cast<float>((row - cy) / fy), z);
    }
  }

  cv::Mat rotation_vector;
  cv::Rodrigues(rotation_, rotation_vector);
  cv::Mat scaled_rgb_camera_matrix = rgb_camera_matrix_.clone();
  const double scale_x =
    static_cast<double>(frame.rgb_width) / calibration_.rgb.width;
  const double scale_y =
    static_cast<double>(frame.rgb_height) / calibration_.rgb.height;
  scaled_rgb_camera_matrix.at<double>(0, 0) *= scale_x;
  scaled_rgb_camera_matrix.at<double>(0, 1) *= scale_x;
  scaled_rgb_camera_matrix.at<double>(0, 2) *= scale_x;
  scaled_rgb_camera_matrix.at<double>(1, 1) *= scale_y;
  scaled_rgb_camera_matrix.at<double>(1, 2) *= scale_y;
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    points, rotation_vector, translation_mm_, scaled_rgb_camera_matrix,
    rgb_distortion_, projected);
  for (size_t index = 0; index < projected.size(); ++index) {
    const int x = static_cast<int>(std::lround(projected[index].x));
    const int y = static_cast<int>(std::lround(projected[index].y));
    if (frame.depth_mm[index] == 0 || x < 0 || y < 0 ||
      x >= static_cast<int>(frame.rgb_width) ||
      y >= static_cast<int>(frame.rgb_height))
    {
      continue;
    }
    const size_t color_index =
      (static_cast<size_t>(y) * frame.rgb_width + x) * 3;
    const uint8_t blue = frame.rgb_bgr[color_index];
    const uint8_t green = frame.rgb_bgr[color_index + 1];
    const uint8_t red = frame.rgb_bgr[color_index + 2];
    frame.mapped_rgba[index] =
      static_cast<uint32_t>(red) |
      (static_cast<uint32_t>(green) << 8U) |
      (static_cast<uint32_t>(blue) << 16U) |
      0xFF000000U;
  }
}

}  // namespace maixsense_a075v
