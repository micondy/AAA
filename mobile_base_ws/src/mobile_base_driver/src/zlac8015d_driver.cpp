#include "mobile_base/zlac8015d_driver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace mobile_base
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int8_t kProfileVelocityMode = 3;
constexpr uint16_t kShutdown = 0x0006;
constexpr uint16_t kSwitchOn = 0x0007;
constexpr uint16_t kEnableOperation = 0x000F;
constexpr uint16_t kOperationEnabledMask = 0x006F;
constexpr uint16_t kOperationEnabledValue = 0x0027;

bool is_sdo_success(uint8_t command)
{
  return command == 0x60 || command == 0x4F || command == 0x4B ||
         command == 0x47 || command == 0x43;
}

bool is_sdo_upload(uint8_t command)
{
  return command == 0x4F || command == 0x4B || command == 0x47 || command == 0x43;
}
}  // namespace

Zlac8015dDriver::Zlac8015dDriver(std::unique_ptr<CanTransport> transport)
: transport_(std::move(transport))
{
  if (!transport_) {throw std::invalid_argument("Zlac8015dDriver 必须提供 CAN 传输对象");}
  transport_->set_receive_callback([this](const CanFrame & frame) {handle_frame(frame);});
}

Zlac8015dDriver::~Zlac8015dDriver()
{
  disable_all();
  close();
}

void Zlac8015dDriver::open(const std::string & interface_name)
{
  transport_->open(interface_name, false);
}

void Zlac8015dDriver::close() noexcept
{
  transport_->close();
}

void Zlac8015dDriver::set_object_dictionary(const ZlacObjectDictionary & dictionary)
{
  dictionary_ = dictionary;
}

void Zlac8015dDriver::add_channel(const ZlacChannelConfig & config)
{
  if (config.node_id == 0 || config.node_id > 127 || config.channel < 1 || config.channel > 2) {
    throw std::invalid_argument("ZLAC8015D 节点 ID 或通道号无效");
  }
  if (!std::isfinite(config.max_motor_rpm) || config.max_motor_rpm <= 0.0 ||
    !std::isfinite(config.gear_ratio) || config.gear_ratio <= 0.0 ||
    !std::isfinite(config.command_units_per_rpm) || config.command_units_per_rpm <= 0.0 ||
    !std::isfinite(config.feedback_rpm_per_unit) || config.feedback_rpm_per_unit <= 0.0)
  {
    throw std::invalid_argument("ZLAC8015D 转速、减速比或单位换算参数无效");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const uint16_t key = channel_key(config.node_id, config.channel);
  configs_[key] = config;
  states_.try_emplace(key);
}

void Zlac8015dDriver::configure_profile_velocity(
  int32_t acceleration, int32_t deceleration, int32_t communication_watchdog_ms,
  std::chrono::milliseconds timeout)
{
  if (acceleration < 0 || acceleration > 32767 || deceleration < 0 ||
    deceleration > 32767)
  {
    throw std::invalid_argument("ZLAC8015D 加减速时间必须在 0~32767 ms 范围内");
  }
  if (communication_watchdog_ms < 0 || communication_watchdog_ms > 32000) {
    throw std::invalid_argument("ZLAC8015D 通信掉线保护时间必须在 0~32000 ms 范围内");
  }

  std::unordered_set<uint8_t> nodes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : configs_) {nodes.insert(item.second.node_id);}
  }
  for (const auto node : nodes) {
    send_nmt(0x01, node);  // 启动指定 CANopen 远程节点。
    if (!write_sdo(
        node, dictionary_.communication_watchdog, 0,
        communication_watchdog_ms, 2, timeout))
    {
      throw std::runtime_error("ZLAC8015D 通信掉线保护时间配置失败");
    }
    if (!write_sdo(node, dictionary_.synchronous_control, 0, 0, 2, timeout)) {
      throw std::runtime_error("ZLAC8015D 拒绝切换到异步控制模式");
    }
    if (!write_sdo(node, dictionary_.mode_of_operation, 0, kProfileVelocityMode, 1, timeout)) {
      throw std::runtime_error("ZLAC8015D 拒绝切换到轮廓速度模式");
    }
    const auto mode = read_sdo(node, dictionary_.mode_display, 0, timeout);
    if (!mode || static_cast<int8_t>(*mode & 0xFFU) != kProfileVelocityMode) {
      throw std::runtime_error("ZLAC8015D 运行模式回读不是轮廓速度模式");
    }
    for (uint8_t channel = 1; channel <= 2; ++channel) {
      if (!write_sdo(node, dictionary_.acceleration, channel, acceleration, 4, timeout) ||
        !write_sdo(node, dictionary_.deceleration, channel, deceleration, 4, timeout))
      {
        throw std::runtime_error("ZLAC8015D 加减速时间配置失败");
      }
    }
  }
}

void Zlac8015dDriver::enable_all(std::chrono::milliseconds timeout)
{
  std::unordered_set<uint8_t> nodes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : configs_) {nodes.insert(item.second.node_id);}
  }
  for (const auto node : nodes) {
    if (!write_sdo(node, dictionary_.controlword, 0, kShutdown, 2, timeout) ||
      !write_sdo(node, dictionary_.controlword, 0, kSwitchOn, 2, timeout) ||
      !write_sdo(node, dictionary_.controlword, 0, kEnableOperation, 2, timeout))
    {
      throw std::runtime_error("ZLAC8015D 的 CiA-402 使能流程失败");
    }
    const auto combined_status = read_sdo(node, dictionary_.statusword, 0, timeout);
    if (!combined_status) {
      throw std::runtime_error("ZLAC8015D 使能后无法读取状态字");
    }
    for (uint8_t channel = 1; channel <= 2; ++channel) {
      const auto status = static_cast<uint16_t>(
        (*combined_status >> (16U * (channel - 1U))) & 0xFFFFU);
      if ((status & kOperationEnabledMask) != kOperationEnabledValue) {
        throw std::runtime_error("ZLAC8015D 使能后至少一个通道未进入 Operation Enabled");
      }
    }
  }
}

void Zlac8015dDriver::disable_all() noexcept
{
  if (!transport_ || !transport_->is_open()) {return;}
  try {
    std::unordered_set<uint8_t> nodes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & item : configs_) {nodes.insert(item.second.node_id);}
    }
    for (const auto node : nodes) {
      // 正常停机使用 Shutdown，让驱动器按 605Bh 的停止方式退出使能。
      write_sdo_async(node, dictionary_.controlword, 0, kShutdown, 2);
    }
  } catch (...) {
  }
}

void Zlac8015dDriver::command_velocity(
  uint8_t node_id, uint8_t channel, double wheel_velocity_rad_s)
{
  ZlacChannelConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config = configs_.at(channel_key(node_id, channel));
  }
  const double requested_motor_rpm =
    wheel_velocity_rad_s * 60.0 / kTwoPi * config.gear_ratio;
  const double motor_rpm = std::clamp(
    requested_motor_rpm, -config.max_motor_rpm, config.max_motor_rpm);
  const auto raw = static_cast<int32_t>(std::lround(
    motor_rpm * config.command_units_per_rpm * config.direction));
  write_sdo_async(node_id, dictionary_.target_velocity, channel, raw, 4);
}

void Zlac8015dDriver::request_velocity(uint8_t node_id, uint8_t channel)
{
  read_sdo_async(node_id, dictionary_.actual_velocity, channel);
}

void Zlac8015dDriver::request_diagnostics(uint8_t node_id)
{
  read_sdo_async(node_id, dictionary_.statusword, 0);
  read_sdo_async(node_id, dictionary_.fault_code, 0);
}

std::optional<ZlacChannelState> Zlac8015dDriver::state(uint8_t node_id, uint8_t channel) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iterator = states_.find(channel_key(node_id, channel));
  return iterator == states_.end() ? std::nullopt : std::optional<ZlacChannelState>(iterator->second);
}

uint16_t Zlac8015dDriver::channel_key(uint8_t node_id, uint8_t channel)
{
  return (static_cast<uint16_t>(node_id) << 8U) | channel;
}

uint32_t Zlac8015dDriver::request_key(uint8_t node_id, uint16_t index, uint8_t subindex)
{
  return (static_cast<uint32_t>(node_id) << 24U) |
    (static_cast<uint32_t>(index) << 8U) | subindex;
}

void Zlac8015dDriver::handle_frame(const CanFrame & frame)
{
  if (frame.size < 8 || frame.id < 0x581 || frame.id > 0x5FF) {return;}
  const uint8_t node_id = static_cast<uint8_t>(frame.id - 0x580U);
  const uint16_t index = static_cast<uint16_t>(frame.data[1]) |
    (static_cast<uint16_t>(frame.data[2]) << 8U);
  const uint8_t subindex = frame.data[3];
  const uint32_t key = request_key(node_id, index, subindex);
  const uint8_t command = frame.data[0];
  uint32_t value = static_cast<uint32_t>(frame.data[4]) |
    (static_cast<uint32_t>(frame.data[5]) << 8U) |
    (static_cast<uint32_t>(frame.data[6]) << 16U) |
    (static_cast<uint32_t>(frame.data[7]) << 24U);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto pending = pending_sdo_.find(key);
    if (pending != pending_sdo_.end()) {
      pending->second.ready = true;
      pending->second.success = is_sdo_success(command);
      pending->second.value = value;
    }

    if (index == dictionary_.actual_velocity && subindex >= 1 && subindex <= 2 &&
      is_sdo_upload(command))
    {
      const uint16_t channel = channel_key(node_id, subindex);
      const auto config = configs_.find(channel);
      if (config != configs_.end()) {
        const auto signed_value = static_cast<int32_t>(value);
        const double motor_rpm = signed_value * config->second.feedback_rpm_per_unit *
          config->second.direction;
        auto & state = states_[channel];
        state.velocity_rad_s = motor_rpm * kTwoPi / 60.0 / config->second.gear_ratio;
        state.online = true;
        state.stamp = frame.stamp;
      }
    }
    if (subindex == 0 && index == dictionary_.statusword && is_sdo_upload(command)) {
      for (uint8_t channel = 1; channel <= 2; ++channel) {
        const auto key_for_channel = channel_key(node_id, channel);
        if (configs_.count(key_for_channel) == 0) {continue;}
        auto & state = states_[key_for_channel];
        state.statusword = static_cast<uint16_t>(
          (value >> (16U * (channel - 1U))) & 0xFFFFU);
        state.diagnostics_valid = true;
      }
    }
    if (subindex == 0 && index == dictionary_.fault_code && is_sdo_upload(command)) {
      for (uint8_t channel = 1; channel <= 2; ++channel) {
        const auto key_for_channel = channel_key(node_id, channel);
        if (configs_.count(key_for_channel) == 0) {continue;}
        states_[key_for_channel].fault_code = value;
      }
    }
  }
  sdo_cv_.notify_all();
}

void Zlac8015dDriver::send_nmt(uint8_t command, uint8_t node_id)
{
  CanFrame frame;
  frame.id = 0x000;
  frame.size = 2;
  frame.data[0] = command;
  frame.data[1] = node_id;
  transport_->send(frame);
}

bool Zlac8015dDriver::write_sdo(
  uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value, uint8_t bytes,
  std::chrono::milliseconds timeout)
{
  const uint32_t key = request_key(node_id, index, subindex);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_sdo_[key] = {};
  }
  write_sdo_async(node_id, index, subindex, value, bytes);
  std::unique_lock<std::mutex> lock(mutex_);
  const bool received = sdo_cv_.wait_for(lock, timeout, [&]() {return pending_sdo_[key].ready;});
  const bool success = received && pending_sdo_[key].success;
  pending_sdo_.erase(key);
  return success;
}

std::optional<uint32_t> Zlac8015dDriver::read_sdo(
  uint8_t node_id, uint16_t index, uint8_t subindex, std::chrono::milliseconds timeout)
{
  const uint32_t key = request_key(node_id, index, subindex);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_sdo_[key] = {};
  }
  read_sdo_async(node_id, index, subindex);
  std::unique_lock<std::mutex> lock(mutex_);
  const bool received = sdo_cv_.wait_for(lock, timeout, [&]() {return pending_sdo_[key].ready;});
  if (!received || !pending_sdo_[key].success) {
    pending_sdo_.erase(key);
    return std::nullopt;
  }
  const uint32_t value = pending_sdo_[key].value;
  pending_sdo_.erase(key);
  return value;
}

void Zlac8015dDriver::write_sdo_async(
  uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value, uint8_t bytes)
{
  CanFrame frame;
  frame.id = 0x600U + node_id;
  frame.size = 8;
  frame.data[0] = bytes == 1 ? 0x2F : (bytes == 2 ? 0x2B : 0x23);
  frame.data[1] = static_cast<uint8_t>(index & 0xFF);
  frame.data[2] = static_cast<uint8_t>(index >> 8U);
  frame.data[3] = subindex;
  const uint32_t raw = static_cast<uint32_t>(value);
  for (uint8_t byte = 0; byte < bytes; ++byte) {
    frame.data[4 + byte] = static_cast<uint8_t>(raw >> (8U * byte));
  }
  transport_->send(frame);
}

void Zlac8015dDriver::read_sdo_async(uint8_t node_id, uint16_t index, uint8_t subindex)
{
  CanFrame frame;
  frame.id = 0x600U + node_id;
  frame.size = 8;
  frame.data[0] = 0x40;
  frame.data[1] = static_cast<uint8_t>(index & 0xFF);
  frame.data[2] = static_cast<uint8_t>(index >> 8U);
  frame.data[3] = subindex;
  transport_->send(frame);
}

}  // namespace mobile_base
