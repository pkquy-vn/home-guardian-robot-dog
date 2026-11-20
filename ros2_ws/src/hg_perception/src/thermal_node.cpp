// SPDX-License-Identifier: Apache-2.0
//
// thermal_node: ROS 2 driver for the Melexis MLX90640 thermal camera (32 x 24 pixels) on Linux I2C.
//
// On start (and after every I2C failure) the node dumps the sensor EEPROM, extracts the
// calibration with the Melexis API, sets the refresh rate and chess mode. A dedicated thread then
// waits for each new subpage, computes object temperatures (emissivity from the parameters,
// reflected temperature = ambient - 8 C), applies the API's bad-pixel correction and publishes the
// full 32 x 24 image on /thermal/image (sensor_msgs/Image, 32FC1, deg C). Slow I2C never blocks
// ROS callbacks. On I2C errors it logs, closes the bus, retries every 2 s and recovers on its own.

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "hg_perception/mlx90640_linux_i2c.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"  // MLX90640_I2CRead(), used to poll the status register.
}

namespace {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

constexpr int kWidth = MLX90640_COLUMN_NUM;  // 32
constexpr int kHeight = MLX90640_LINE_NUM;   // 24
constexpr int kPixels = MLX90640_PIXEL_NUM;  // 768
// MLX90640_GetFrameData() output: 768 pixels, 64 auxiliary words, control register, subpage.
constexpr int kFrameWords = MLX90640_PIXEL_NUM + MLX90640_AUX_NUM + 2;
constexpr float kTaShift = 8.0f;  // Reflected temperature = ambient - 8 C (sensor in open air).
constexpr int kChessMode = 1;     // MLX90640_GetCurMode(): 1 = chess, 0 = interleaved.
constexpr auto kRetryPeriod = 2s;
constexpr auto kStatusPollPeriod = 4ms;
constexpr auto kStillFailingLogPeriod = 30s;
constexpr int kMaxConsecutiveFrameErrors = 10;
constexpr bool kHostIsBigEndian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;

// Control-register codes for the refresh rate (subpages per second).
std::optional<uint8_t> refresh_code(double hz) {
  const double rates[] = {0.5, 1, 2, 4, 8, 16, 32, 64};
  for (uint8_t code = 0; code < 8; ++code) {
    if (std::fabs(hz - rates[code]) < 1e-6) return code;
  }
  return std::nullopt;
}

std::string describe_api_error(int code) {
  switch (-code) {
    case MLX90640_I2C_NACK_ERROR:
      return std::string("I2C transfer failed: ") + std::strerror(hg::mlx90640_i2c::last_errno());
    case MLX90640_I2C_WRITE_ERROR:
      return "I2C write did not read back the written value";
    case MLX90640_FRAME_DATA_ERROR:
      return "invalid frame data";
    default:
      return "MLX90640 API error " + std::to_string(code);
  }
}

std::string hex_address(uint8_t address) {
  char text[8];
  std::snprintf(text, sizeof(text), "0x%02X", address);
  return text;
}

double seconds_since(SteadyClock::time_point start) {
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

}  // namespace

class ThermalNode : public rclcpp::Node {
 public:
  ThermalNode();
  ~ThermalNode() override;

 private:
  double declare_number(const std::string& name, double default_value);
  bool stopping() const { return stop_.load() || !rclcpp::ok(); }

  void run();
  bool connect(std::string* error);
  bool wait_for_subpage(std::string* error);
  bool read_subpage(std::string* error, bool* reconnect);
  void publish_image();
  void log_stats(const std::array<float, kPixels>& image);
  void report_failure(const std::string& error);
  void sleep_before_retry();

  // Parameters (read once at start).
  std::string i2c_device_;
  uint8_t address_ = 0x33;
  double refresh_rate_hz_ = 8.0;
  uint8_t refresh_code_ = 0x04;
  float emissivity_ = 0.95f;
  bool flip_horizontal_ = false;
  bool flip_vertical_ = false;
  std::string frame_id_;
  bool log_stats_ = true;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Acquisition state: used only by the worker thread.
  paramsMLX90640 params_{};
  uint16_t eeprom_[MLX90640_EEPROM_DUMP_NUM] = {};
  uint16_t frame_[kFrameWords] = {};
  std::array<float, kPixels> temps_{};
  bool sensor_ready_ = false;
  bool connected_once_ = false;
  int subpages_since_connect_ = 0;
  int consecutive_frame_errors_ = 0;
  bool failing_ = false;
  SteadyClock::time_point failing_since_;
  SteadyClock::time_point last_failure_log_;
  bool stats_started_ = false;
  SteadyClock::time_point stats_window_start_;
  int stats_count_ = 0;

  std::atomic<bool> stop_{false};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
};

ThermalNode::ThermalNode() : Node("thermal_node") {
  i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  const int64_t address = declare_parameter<int64_t>("address", 0x33);
  refresh_rate_hz_ = declare_number("refresh_rate_hz", 8.0);
  const double emissivity = declare_number("emissivity", 0.95);
  flip_horizontal_ = declare_parameter<bool>("flip_horizontal", false);
  flip_vertical_ = declare_parameter<bool>("flip_vertical", false);
  frame_id_ = declare_parameter<std::string>("frame_id", "thermal_link");
  log_stats_ = declare_parameter<bool>("log_stats", true);

  if (address < 0x08 || address > 0x77) {
    throw std::invalid_argument("address must be a 7-bit I2C address (0x08 to 0x77), got " +
                                std::to_string(address));
  }
  address_ = static_cast<uint8_t>(address);
  const std::optional<uint8_t> code = refresh_code(refresh_rate_hz_);
  if (!code) {
    throw std::invalid_argument("refresh_rate_hz must be 0.5, 1, 2, 4, 8, 16, 32 or 64, got " +
                                std::to_string(refresh_rate_hz_));
  }
  refresh_code_ = *code;
  if (!(emissivity > 0.0 && emissivity <= 1.0)) {
    throw std::invalid_argument("emissivity must be in (0, 1], got " + std::to_string(emissivity));
  }
  emissivity_ = static_cast<float>(emissivity);
  if (refresh_rate_hz_ > 16.0) {
    RCLCPP_WARN(get_logger(), "%g Hz needs a faster I2C bus than 400 kHz; expect read errors",
                refresh_rate_hz_);
  }

  publisher_ = create_publisher<sensor_msgs::msg::Image>("/thermal/image", rclcpp::QoS(5));

  RCLCPP_INFO(get_logger(),
              "MLX90640 on %s at %s: %g Hz, emissivity %.2f, flip horizontal %s, vertical %s, "
              "frame %s",
              i2c_device_.c_str(), hex_address(address_).c_str(), refresh_rate_hz_, emissivity,
              flip_horizontal_ ? "on" : "off", flip_vertical_ ? "on" : "off", frame_id_.c_str());

  worker_ = std::thread([this] { run(); });
}

ThermalNode::~ThermalNode() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) worker_.join();
  hg::mlx90640_i2c::close_device();
}

double ThermalNode::declare_number(const std::string& name, double default_value) {
  // Accept 8 as well as 8.0 in YAML and on the command line.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  const rclcpp::ParameterValue value =
      declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(value.get<int64_t>());
  }
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return value.get<double>();
  throw std::invalid_argument("parameter " + name + " must be a number");
}

// ---------------------------------------------------------------------------------------------
// Acquisition thread
// ---------------------------------------------------------------------------------------------

void ThermalNode::run() {
  try {
    while (!stopping()) {
      std::string error;
      if (!sensor_ready_ && !connect(&error)) {
        if (stopping()) break;
        report_failure(error);
        sleep_before_retry();
        continue;
      }
      bool reconnect = true;
      if (!read_subpage(&error, &reconnect)) {
        if (stopping()) break;
        if (reconnect) {
          hg::mlx90640_i2c::close_device();
          sensor_ready_ = false;
          report_failure(error);
          sleep_before_retry();
        }
        continue;
      }
      // Chess mode updates half of the pixels per subpage: publish once both have been read.
      if (subpages_since_connect_ >= 2) publish_image();
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "thermal acquisition thread stopped: %s", e.what());
  }
  hg::mlx90640_i2c::close_device();
}

bool ThermalNode::connect(std::string* error) {
  const std::string where = " at " + hex_address(address_) + " on " + i2c_device_;
  if (!hg::mlx90640_i2c::open_device(i2c_device_)) {
    const int err = hg::mlx90640_i2c::last_errno();
    std::string hint;
    if (err == ENOENT) hint = " (is I2C enabled in /boot/firmware/config.txt?)";
    if (err == EACCES) hint = " (is this user in the i2c group?)";
    *error = "cannot open " + i2c_device_ + ": " + std::strerror(err) + hint;
    return false;
  }

  int rc = MLX90640_DumpEE(address_, eeprom_);
  if (rc != MLX90640_NO_ERROR) {
    *error = "no response from the MLX90640" + where + " while reading its EEPROM (" +
             describe_api_error(rc) + "); check wiring and power";
    hg::mlx90640_i2c::close_device();
    return false;
  }

  rc = MLX90640_ExtractParameters(eeprom_, &params_);
  if (rc != MLX90640_NO_ERROR && !connected_once_) {
    // The calibration is still extracted; the code reports too many or adjacent bad pixels.
    RCLCPP_WARN(get_logger(), "MLX90640 EEPROM reports a bad-pixel problem (code %d)", rc);
  }

  rc = MLX90640_SetRefreshRate(address_, refresh_code_);
  if (rc == MLX90640_NO_ERROR) rc = MLX90640_SetChessMode(address_);
  if (rc != MLX90640_NO_ERROR) {
    *error = "configuring the MLX90640" + where + " failed (" + describe_api_error(rc) + ")";
    hg::mlx90640_i2c::close_device();
    return false;
  }
  const int mode = MLX90640_GetCurMode(address_);
  const int rate = MLX90640_GetRefreshRate(address_);
  if (mode != kChessMode || rate != refresh_code_) {
    *error = mode < 0 || rate < 0
                 ? "reading back the MLX90640 configuration" + where + " failed (" +
                       describe_api_error(mode < 0 ? mode : rate) + ")"
                 : "the MLX90640" + where + " did not accept chess mode and the refresh rate";
    hg::mlx90640_i2c::close_device();
    return false;
  }

  temps_.fill(std::numeric_limits<float>::quiet_NaN());
  subpages_since_connect_ = 0;
  consecutive_frame_errors_ = 0;
  stats_started_ = false;
  sensor_ready_ = true;

  if (!connected_once_) {
    connected_once_ = true;
    const auto count = [](const uint16_t* pixels) {
      return static_cast<int>(
          std::count_if(pixels, pixels + 5, [](uint16_t p) { return p != 0xFFFF; }));
    };
    RCLCPP_INFO(get_logger(),
                "MLX90640 ready%s: %g Hz, chess mode, %d broken and %d outlier pixels corrected",
                where.c_str(), refresh_rate_hz_, count(params_.brokenPixels),
                count(params_.outlierPixels));
  }
  return true;
}

bool ThermalNode::wait_for_subpage(std::string* error) {
  // Poll the status register with short sleeps instead of letting MLX90640_GetFrameData() spin on
  // the bus. Give up after four subpage periods (at least 1 s).
  const std::chrono::duration<double> period(1.0 / refresh_rate_hz_);
  const std::chrono::duration<double> timeout =
      std::max<std::chrono::duration<double>>(std::chrono::duration<double>(1.0), 4.0 * period);
  const auto deadline =
      SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(timeout);
  while (!stopping()) {
    uint16_t status = 0;
    const int rc = MLX90640_I2CRead(address_, MLX90640_STATUS_REG, 1, &status);
    if (rc != MLX90640_NO_ERROR) {
      *error = "reading the MLX90640 status register failed (" + describe_api_error(rc) + ")";
      return false;
    }
    if (MLX90640_GET_DATA_READY(status)) return true;
    if (SteadyClock::now() >= deadline) {
      *error = "no new subpage from the MLX90640 within " + std::to_string(timeout.count()) + " s";
      return false;
    }
    std::this_thread::sleep_for(kStatusPollPeriod);
  }
  return false;
}

bool ThermalNode::read_subpage(std::string* error, bool* reconnect) {
  *reconnect = true;
  if (!wait_for_subpage(error)) return false;

  const int subpage = MLX90640_GetFrameData(address_, frame_);
  if (subpage < 0) {
    *error = "reading MLX90640 frame data failed (" + describe_api_error(subpage) + ")";
    if (subpage == -MLX90640_FRAME_DATA_ERROR &&
        ++consecutive_frame_errors_ < kMaxConsecutiveFrameErrors) {
      // An occasional invalid frame is skipped; only a run of them triggers a reconnect.
      *reconnect = false;
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "%s; skipping it", error->c_str());
    }
    return false;
  }
  consecutive_frame_errors_ = 0;

  const float tr = MLX90640_GetTa(frame_, &params_) - kTaShift;
  MLX90640_CalculateTo(frame_, &params_, emissivity_, tr, temps_.data());
  MLX90640_BadPixelsCorrection(params_.brokenPixels, temps_.data(), kChessMode, &params_);
  MLX90640_BadPixelsCorrection(params_.outlierPixels, temps_.data(), kChessMode, &params_);
  if (subpages_since_connect_ < 2) ++subpages_since_connect_;

  if (failing_) {
    failing_ = false;
    RCLCPP_INFO(get_logger(), "MLX90640 recovered after %.0f s", seconds_since(failing_since_));
  }
  return true;
}

void ThermalNode::publish_image() {
  // Row-major, row 0 at the top. The flips are applied here, so every consumer sees the same
  // orientation.
  std::array<float, kPixels> image;
  for (int row = 0; row < kHeight; ++row) {
    const int src_row = flip_vertical_ ? kHeight - 1 - row : row;
    for (int col = 0; col < kWidth; ++col) {
      const int src_col = flip_horizontal_ ? kWidth - 1 - col : col;
      image[row * kWidth + col] = temps_[src_row * kWidth + src_col];
    }
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = now();
  msg->header.frame_id = frame_id_;
  msg->height = kHeight;
  msg->width = kWidth;
  msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg->is_bigendian = kHostIsBigEndian ? 1 : 0;
  msg->step = static_cast<uint32_t>(kWidth * sizeof(float));
  msg->data.resize(sizeof(image));
  std::memcpy(msg->data.data(), image.data(), sizeof(image));
  publisher_->publish(std::move(msg));

  if (log_stats_) log_stats(image);
}

void ThermalNode::log_stats(const std::array<float, kPixels>& image) {
  const auto now = SteadyClock::now();
  if (!stats_started_) {
    stats_started_ = true;
    stats_window_start_ = now;
    stats_count_ = 0;
    return;
  }
  ++stats_count_;
  const double elapsed = std::chrono::duration<double>(now - stats_window_start_).count();
  if (elapsed < 1.0) return;

  float min = std::numeric_limits<float>::infinity();
  float max = -std::numeric_limits<float>::infinity();
  double sum = 0.0;
  int finite = 0;
  for (float t : image) {
    if (!std::isfinite(t)) continue;
    min = std::min(min, t);
    max = std::max(max, t);
    sum += t;
    ++finite;
  }
  // Even width and height: the centre is the mean of the four middle pixels.
  const int r = kHeight / 2;
  const int c = kWidth / 2;
  const float center = (image[(r - 1) * kWidth + c - 1] + image[(r - 1) * kWidth + c] +
                        image[r * kWidth + c - 1] + image[r * kWidth + c]) /
                       4.0f;
  RCLCPP_INFO(get_logger(), "min %.1f C, max %.1f C, mean %.1f C, center %.1f C, %.1f Hz", min, max,
              finite > 0 ? sum / finite : std::nan(""), center, stats_count_ / elapsed);
  stats_window_start_ = now;
  stats_count_ = 0;
}

void ThermalNode::report_failure(const std::string& error) {
  const auto now = SteadyClock::now();
  if (!failing_) {
    failing_ = true;
    failing_since_ = now;
    last_failure_log_ = now;
    RCLCPP_ERROR(get_logger(), "%s; retrying every %lld s", error.c_str(),
                 static_cast<long long>(kRetryPeriod.count()));
  } else if (now - last_failure_log_ >= kStillFailingLogPeriod) {
    last_failure_log_ = now;
    RCLCPP_WARN(get_logger(), "MLX90640 still failing after %.0f s: %s",
                seconds_since(failing_since_), error.c_str());
  }
}

void ThermalNode::sleep_before_retry() {
  std::unique_lock<std::mutex> lock(mutex_);
  wake_.wait_for(lock, kRetryPeriod, [this] { return stop_.load(); });
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<ThermalNode>();
    rclcpp::spin(node);  // No callbacks of its own; acquisition runs in the node's thread.
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("thermal_node"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
