#include "maixsense_a075v/device.hpp"

#include <curl/curl.h>

#include <array>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "frame_pipeline.hpp"

namespace maixsense_a075v
{
namespace
{

constexpr size_t kModuleInfoSize = 81;
constexpr size_t kDepthLutSize = 65536;
constexpr auto kReadyRetryPeriod = std::chrono::milliseconds(200);

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
#pragma pack(pop)

static_assert(sizeof(WireConfig) == 12);

size_t append_response(
  char * data, size_t size, size_t count, void * context)
{
  const size_t byte_count = size * count;
  auto * response = static_cast<std::vector<uint8_t> *>(context);
  const auto * begin = reinterpret_cast<uint8_t *>(data);
  response->insert(response->end(), begin, begin + byte_count);
  return byte_count;
}

std::once_flag curl_init_flag;

}  // namespace

class Device::Impl
{
public:
  Impl(
    std::string host, uint16_t port,
    std::chrono::milliseconds request_timeout)
  : base_url_(
      "http://" + std::move(host) + ":" + std::to_string(port)),
    request_timeout_(request_timeout)
  {
    std::call_once(curl_init_flag, []() {curl_global_init(CURL_GLOBAL_DEFAULT);});
  }

  bool request(
    const char * path, const uint8_t * body, size_t body_size,
    std::vector<uint8_t> & response)
  {
    response.clear();
    CURL * handle = curl_easy_init();
    if (handle == nullptr) {
      error_ = "could not create a libcurl easy handle";
      return false;
    }

    const std::string url = base_url_ + path;
    std::array<char, CURL_ERROR_SIZE> curl_error{};
    curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, request_timeout_.count());
    curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, append_response);
    curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, curl_error.data());
    if (body != nullptr) {
      curl_easy_setopt(handle, CURLOPT_POST, 1L);
      curl_easy_setopt(handle, CURLOPT_POSTFIELDS, body);
      curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE, body_size);
    }

    const CURLcode result = curl_easy_perform(handle);
    long status = 0;
    curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
    curl_easy_cleanup(handle);

    if (result != CURLE_OK) {
      const char * detail =
        curl_error[0] == '\0' ? curl_easy_strerror(result) : curl_error.data();
      error_ = std::string("HTTP ") + path + " failed: " + detail;
      return false;
    }
    if (status < 200 || status >= 300) {
      error_ = std::string("HTTP ") + path + " returned status " +
        std::to_string(status);
      return false;
    }
    error_.clear();
    return true;
  }

  bool get(const char * path, std::vector<uint8_t> & response)
  {
    return request(path, nullptr, 0, response);
  }

  std::string base_url_;
  std::chrono::milliseconds request_timeout_;
  FramePipeline pipeline_;
  Calibration calibration_;
  std::string error_;
  std::vector<uint8_t> module_info_;
};

Device::Device(
  std::string host, uint16_t port,
  std::chrono::milliseconds request_timeout)
: impl_(std::make_unique<Impl>(
    std::move(host), port, request_timeout))
{
}

Device::~Device() = default;
Device::Device(Device &&) noexcept = default;
Device & Device::operator=(Device &&) noexcept = default;

bool Device::wait_until_ready(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    std::vector<uint8_t> response;
    if (impl_->get("/getinfo", response)) {
      if (response.size() == kModuleInfoSize) {
        impl_->module_info_ = std::move(response);
        return true;
      }
      impl_->error_ = "HTTP /getinfo returned " +
        std::to_string(response.size()) + " bytes; expected " +
        std::to_string(kModuleInfoSize);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(kReadyRetryPeriod);
  } while (true);
  return false;
}

bool Device::load_calibration()
{
  if (impl_->module_info_.empty() &&
    !impl_->get("/getinfo", impl_->module_info_))
  {
    return false;
  }
  if (!impl_->pipeline_.set_module_info(
      impl_->module_info_, impl_->error_))
  {
    return false;
  }

  std::vector<uint8_t> response;
  if (!impl_->get("/get_lut", response)) {
    return false;
  }
  if (response.size() < kDepthLutSize) {
    impl_->error_ = "HTTP /get_lut returned " +
      std::to_string(response.size()) + " bytes; expected at least " +
      std::to_string(kDepthLutSize);
    return false;
  }
  if (!impl_->pipeline_.set_depth_lut(response, impl_->error_)) {
    return false;
  }

  if (!impl_->get("/CameraParms.json", response)) {
    return false;
  }
  if (!impl_->pipeline_.set_rgb_calibration(response, impl_->error_)) {
    return false;
  }
  impl_->calibration_ = impl_->pipeline_.calibration();
  return true;
}

bool Device::set_acquisition_config(const AcquisitionConfig & config)
{
  const WireConfig wire{
    config.trigger_mode,
    config.depth_mode,
    config.depth_shift,
    config.infrared_mode,
    config.status_mode,
    config.status_mask,
    config.rgb_mode,
    config.rgb_resolution,
    config.exposure_time};
  std::vector<uint8_t> response;
  return impl_->request(
    "/set_cfg", reinterpret_cast<const uint8_t *>(&wire), sizeof(wire),
    response);
}

void Device::set_pipeline_config(const PipelineConfig & config)
{
  impl_->pipeline_.set_config(config);
}

bool Device::read_frame(DecodedFrame & frame)
{
  std::vector<uint8_t> response;
  if (!impl_->get("/getdeep", response)) {
    return false;
  }
  return impl_->pipeline_.decode(response, frame, impl_->error_);
}

const Calibration & Device::calibration() const
{
  return impl_->calibration_;
}

const std::string & Device::error() const
{
  return impl_->error_;
}

}  // namespace maixsense_a075v
