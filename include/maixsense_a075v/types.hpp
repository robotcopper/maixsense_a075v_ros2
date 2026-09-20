#ifndef MAIXSENSE_A075V__TYPES_HPP_
#define MAIXSENSE_A075V__TYPES_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace maixsense_a075v
{

inline constexpr uint32_t kDepthWidth = 320;
inline constexpr uint32_t kDepthHeight = 240;
inline constexpr uint32_t kRgbWidth = 800;
inline constexpr uint32_t kRgbHeight = 600;

// Position of a sensor in the body frame, in metres.
struct SensorMount
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Mechanical layout read from the manufacturer STEP model
// (asset/3d_maixsense_a075v.step) and the datasheet. The body frame has its
// origin on the rear face, on the axis of the circular boss of the front face,
// with x out of the lenses, y left and z up. The front face is kBodyDepthM
// ahead of that origin. Both sensors are soldered on the same board, so their
// optical axes are parallel to x; the residual misalignment is in the factory
// calibration, not here.
inline constexpr double kBodyDepthM = 0.0225;
inline constexpr double kBodySideM = 0.036;
inline constexpr SensorMount kTofSensorMount{0.0158, 0.0055, -0.0055};
inline constexpr SensorMount kRgbSensorMount{0.0157, -0.0182, 0.0057};

enum class SpatialFilterType : uint8_t
{
  kGaussian = 0,
  kBilateral = 1,
};

struct AcquisitionConfig
{
  uint8_t trigger_mode{0};
  uint8_t depth_mode{0};
  uint8_t depth_shift{255};
  uint8_t infrared_mode{0};
  uint8_t status_mode{2};
  uint8_t status_mask{7};
  uint8_t rgb_mode{1};
  uint8_t rgb_resolution{0};
  int32_t exposure_time{0};
};

struct PipelineConfig
{
  bool temporal_filter{true};
  double temporal_alpha{0.5};
  bool spatial_filter{true};
  SpatialFilterType spatial_filter_type{SpatialFilterType::kGaussian};
  int spatial_kernel_size{7};
  bool flying_point_filter{true};
  double flying_point_threshold{0.03};
  bool keep_normal{true};
  bool keep_overexposed{false};
  bool keep_underexposed{false};
  bool keep_bad{false};
};

struct CameraCalibration
{
  uint32_t width{0};
  uint32_t height{0};
  std::array<double, 9> camera_matrix{};
  std::array<double, 5> distortion{};
};

struct Calibration
{
  CameraCalibration tof;
  CameraCalibration rgb;
  // Transform from the ToF optical frame to the RGB optical frame.
  std::array<double, 9> tof_to_rgb_rotation{};
  std::array<double, 3> tof_to_rgb_translation_m{};
};

struct DecodedFrame
{
  uint64_t sequence{0};
  uint64_t device_stamp_ms{0};
  uint32_t rgb_width{0};
  uint32_t rgb_height{0};
  std::vector<uint16_t> depth_mm;
  std::vector<uint16_t> intensity;
  std::vector<uint16_t> status;
  std::vector<uint8_t> rgb_bgr;
  std::vector<uint32_t> mapped_rgba;
};

}  // namespace maixsense_a075v

#endif  // MAIXSENSE_A075V__TYPES_HPP_
