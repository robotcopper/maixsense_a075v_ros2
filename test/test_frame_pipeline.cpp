#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "frame_pipeline.hpp"

namespace
{

#pragma pack(push, 1)
struct TestLens
{
  uint8_t calibration_mode{0};
  float fx{225.0F};
  float fy{226.0F};
  float cx{160.0F};
  float cy{120.0F};
  float k1{0.0F};
  float k2{0.0F};
  float k3{0.0F};
  float p1{0.0F};
  float p2{0.0F};
  float skew{0.0F};
};

struct TestModuleInfo
{
  char identity[40]{};
  TestLens lens;
};

struct TestConfig
{
  uint8_t trigger_mode{1};
  uint8_t depth_mode{0};
  uint8_t depth_shift{255};
  uint8_t infrared_mode{0};
  uint8_t status_mode{2};
  uint8_t status_mask{7};
  uint8_t rgb_mode{2};
  uint8_t rgb_resolution{0};
  int32_t exposure_time{0};
};

struct TestFrameHeader
{
  uint64_t sequence{42};
  uint64_t stamp_ms{1234};
  TestConfig config;
  int32_t non_rgb_size{0};
  int32_t rgb_size{0};
};
#pragma pack(pop)

static_assert(sizeof(TestModuleInfo) == 81);
static_assert(sizeof(TestFrameHeader) == 36);

std::vector<uint8_t> bytes(const void * data, size_t size)
{
  const auto * begin = static_cast<const uint8_t *>(data);
  return std::vector<uint8_t>(begin, begin + size);
}

}  // namespace

TEST(FramePipeline, ParsesCalibrationInSiUnits)
{
  maixsense_a075v::FramePipeline pipeline;
  std::string error;
  const TestModuleInfo info{};
  ASSERT_TRUE(pipeline.set_module_info(bytes(&info, sizeof(info)), error))
    << error;

  const std::string json = R"({
    "R_Matrix_data": [1,0,0,0,1,0,0,0,1],
    "T_Vec_data": [-10,20,-30],
    "Camera_Matrix_data": [500,0,400,0,501,300,0,0,1],
    "Distortion_Parm_data": [0.1,0.2,0.01,0.02,0.3]
  })";
  ASSERT_TRUE(
    pipeline.set_rgb_calibration(
      std::vector<uint8_t>(json.begin(), json.end()), error)) << error;

  const auto & calibration = pipeline.calibration();
  EXPECT_DOUBLE_EQ(calibration.tof.camera_matrix[0], 225.0);
  EXPECT_DOUBLE_EQ(calibration.rgb.camera_matrix[4], 501.0);
  EXPECT_DOUBLE_EQ(calibration.tof_to_rgb_translation_m[0], 0.01);
  EXPECT_DOUBLE_EQ(calibration.tof_to_rgb_translation_m[1], -0.02);
  EXPECT_DOUBLE_EQ(calibration.tof_to_rgb_translation_m[2], 0.03);
}

TEST(FramePipeline, DecodesAFrameWithoutRgb)
{
  maixsense_a075v::FramePipeline pipeline;
  std::string error;
  const TestModuleInfo info{};
  ASSERT_TRUE(pipeline.set_module_info(bytes(&info, sizeof(info)), error))
    << error;
  std::vector<uint8_t> lut(65536);
  ASSERT_TRUE(pipeline.set_depth_lut(lut, error)) << error;

  constexpr size_t pixel_count =
    maixsense_a075v::kDepthWidth * maixsense_a075v::kDepthHeight;
  const size_t non_rgb_size =
    pixel_count * sizeof(uint16_t) * 2 + pixel_count;
  TestFrameHeader header;
  header.non_rgb_size = static_cast<int32_t>(non_rgb_size);
  std::vector<uint8_t> wire(sizeof(header) + non_rgb_size);
  std::memcpy(wire.data(), &header, sizeof(header));

  auto * depth = reinterpret_cast<uint16_t *>(wire.data() + sizeof(header));
  auto * infrared = depth + pixel_count;
  auto * status = reinterpret_cast<uint8_t *>(infrared + pixel_count);
  for (size_t index = 0; index < pixel_count; ++index) {
    depth[index] = 1000;
    infrared[index] = 200;
    status[index] = 0;
  }

  maixsense_a075v::PipelineConfig config;
  config.temporal_filter = false;
  config.spatial_filter = false;
  config.flying_point_filter = false;
  pipeline.set_config(config);

  maixsense_a075v::DecodedFrame frame;
  ASSERT_TRUE(pipeline.decode(wire, frame, error)) << error;
  EXPECT_EQ(frame.sequence, 42U);
  EXPECT_EQ(frame.device_stamp_ms, 1234U);
  ASSERT_EQ(frame.depth_mm.size(), pixel_count);
  EXPECT_EQ(frame.depth_mm[pixel_count / 2], 1000);
  EXPECT_EQ(frame.intensity[pixel_count / 2], 200);
  EXPECT_TRUE(frame.rgb_bgr.empty());
}
