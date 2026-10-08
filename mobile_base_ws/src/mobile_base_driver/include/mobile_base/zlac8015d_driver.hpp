#pragma once

#include "mobile_base/can_transport.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace mobile_base
{

struct ZlacObjectDictionary
{
  uint16_t communication_watchdog{0x2000};
  uint16_t synchronous_control{0x200F};
  uint16_t fault_code{0x603F};
  uint16_t controlword{0x6040};
  uint16_t statusword{0x6041};
  uint16_t mode_of_operation{0x6060};
  uint16_t mode_display{0x6061};
  uint16_t target_velocity{0x60FF};
  uint16_t actual_velocity{0x606C};
  uint16_t acceleration{0x6083};
  uint16_t deceleration{0x6084};
};

struct ZlacChannelConfig
{
  std::string joint_name;
  uint8_t node_id{1};
  uint8_t channel{1};
  double direction{1.0};
  double gear_ratio{1.0};
  double max_motor_rpm{1000.0};
  double command_units_per_rpm{1.0};
  double feedback_rpm_per_unit{0.1};
};

struct ZlacChannelState
{
  double velocity_rad_s{0.0};
  uint16_t statusword{0};
  uint32_t fault_code{0};
  bool diagnostics_valid{false};
  bool online{false};
  std::chrono::steady_clock::time_point stamp{};
};

class Zlac8015dDriver
{
public:
  explicit Zlac8015dDriver(std::unique_ptr<CanTransport> transport);
  ~Zlac8015dDriver();

  void open(const std::string & interface_name);
  void close() noexcept;
  void set_object_dictionary(const ZlacObjectDictionary & dictionary);
  void add_channel(const ZlacChannelConfig & config);
  void configure_profile_velocity(int32_t acceleration, int32_t deceleration,
    int32_t communication_watchdog_ms,
    std::chrono::milliseconds timeout);
  void enable_all(std::chrono::milliseconds timeout);
  void disable_all() noexcept;
  void command_velocity(uint8_t node_id, uint8_t channel, double wheel_velocity_rad_s);
  void request_velocity(uint8_t node_id, uint8_t channel);
  void request_diagnostics(uint8_t node_id);
  std::optional<ZlacChannelState> state(uint8_t node_id, uint8_t channel) const;

private:
  struct SdoReply
  {
    bool ready{false};
    bool success{false};
    uint32_t value{0};
  };

  static uint16_t channel_key(uint8_t node_id, uint8_t channel);
  static uint32_t request_key(uint8_t node_id, uint16_t index, uint8_t subindex);
  void handle_frame(const CanFrame & frame);
  void send_nmt(uint8_t command, uint8_t node_id);
  bool write_sdo(uint8_t node_id, uint16_t index, uint8_t subindex,
    int32_t value, uint8_t bytes, std::chrono::milliseconds timeout);
  std::optional<uint32_t> read_sdo(uint8_t node_id, uint16_t index, uint8_t subindex,
    std::chrono::milliseconds timeout);
  void write_sdo_async(uint8_t node_id, uint16_t index, uint8_t subindex,
    int32_t value, uint8_t bytes);
  void read_sdo_async(uint8_t node_id, uint16_t index, uint8_t subindex);

  std::unique_ptr<CanTransport> transport_;
  ZlacObjectDictionary dictionary_{};
  std::unordered_map<uint16_t, ZlacChannelConfig> configs_;
  mutable std::mutex mutex_;
  std::condition_variable sdo_cv_;
  std::unordered_map<uint16_t, ZlacChannelState> states_;
  std::unordered_map<uint32_t, SdoReply> pending_sdo_;
};

}  // namespace mobile_base
