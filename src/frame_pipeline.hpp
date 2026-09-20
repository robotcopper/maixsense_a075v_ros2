#ifndef MAIXSENSE_A075V__FRAME_PIPELINE_HPP_
#define MAIXSENSE_A075V__FRAME_PIPELINE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "maixsense_a075v/types.hpp"

namespace maixsense_a075v
{

class FramePipeline
{
public:
  FramePipeline();

  bool set_module_info(const std::vector<uint8_t> & data, std::string & error);
  bool set_depth_lut(const std::vector<uint8_t> & data, std::string & error);
  bool set_rgb_calibration(
    const std::vector<uint8_t> & json, std::string & error);
  void set_config(const PipelineConfig & config);
  bool decode(
    const std::vector<uint8_t> & payload, DecodedFrame & frame,
    std::string & error);

  const Calibration & calibration() const;

private:
  void reset_temporal_filter();
  void apply_filters(std::vector<uint16_t> & depth);
  void rectify(
    const std::vector<uint16_t> & input, std::vector<uint16_t> & output) const;
  void map_rgb_to_tof(DecodedFrame & frame) const;

  Calibration calibration_;
  PipelineConfig config_;
  std::array<uint16_t, 256> depth_lut_{};
  std::vector<uint16_t> previous_depth_;
  cv::Mat rectify_map_x_;
  cv::Mat rectify_map_y_;
  cv::Mat rotation_;
  cv::Mat translation_mm_;
  cv::Mat rgb_camera_matrix_;
  cv::Mat rgb_distortion_;
  bool module_info_ready_{false};
  bool lut_ready_{false};
  bool rgb_calibration_ready_{false};
};

}  // namespace maixsense_a075v

#endif  // MAIXSENSE_A075V__FRAME_PIPELINE_HPP_
