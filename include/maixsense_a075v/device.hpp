#ifndef MAIXSENSE_A075V__DEVICE_HPP_
#define MAIXSENSE_A075V__DEVICE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "maixsense_a075v/types.hpp"

namespace maixsense_a075v
{

class Device
{
public:
  Device(std::string host, uint16_t port, std::chrono::milliseconds request_timeout);
  ~Device();

  Device(const Device &) = delete;
  Device & operator=(const Device &) = delete;
  Device(Device &&) noexcept;
  Device & operator=(Device &&) noexcept;

  bool wait_until_ready(std::chrono::milliseconds timeout);
  bool load_calibration();
  bool set_acquisition_config(const AcquisitionConfig & config);
  void set_pipeline_config(const PipelineConfig & config);
  bool read_frame(DecodedFrame & frame);

  const Calibration & calibration() const;
  const std::string & error() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maixsense_a075v

#endif  // MAIXSENSE_A075V__DEVICE_HPP_
