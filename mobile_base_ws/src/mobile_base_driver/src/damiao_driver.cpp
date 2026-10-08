#include "mobile_base/damiao_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace mobile_base
{
namespace
{
constexpr uint8_t kPmaxRegister = 0x15;
constexpr uint8_t kVmaxRegister = 0x16;
constexpr uint8_t kTmaxRegister = 0x17;
constexpr uint8_t kParameterRead = 0x33;
constexpr uint8_t kRefresh = 0xCC;
constexpr uint8_t kEnable = 0xFC;
constexpr uint8_t kDisable = 0xFD;
constexpr double kTwoPi = 6.28318530717958647692;

double uint_to_float(uint32_t value, double minimum, double maximum, unsigned bits)
{
  const double span = maximum - minimum;
  return static_cast<double>(value) * span / static_cast<double>((1UL << bits) - 1UL) + minimum;
}

uint32_t parameter_key(uint8_t motor_id, uint8_t register_id)
{
  return (static_cast<uint32_t>(motor_id) << 16U) | register_id;
}
}  // namespace

DamiaoDriver::DamiaoDriver(std::unique_ptr<CanTransport> transport)
: transport_(std::move(transport))
{
  if (!transport_) {
    throw std::invalid_argument("DamiaoDriver 必须提供 CAN 传输对象");
  }
  transport_->set_receive_callback([this](const CanFrame & frame) {handle_frame(frame);});
}

DamiaoDriver::~DamiaoDriver()
{
  disable_all();
  close();
}

void DamiaoDriver::open(const std::string & interface_name, bool can_fd)
{
  transport_->open(interface_name, can_fd);
}

void DamiaoDriver::close() noexcept
{
  transport_->close();
}

void DamiaoDriver::add_motor(const DmMotorConfig & config)
{
  if (config.motor_id == 0 || config.motor_id > 0x0F) {
    throw std::invalid_argument("达妙电机 ID 必须在 [1, 15] 范围内");
  }
  if (config.master_id > 0x7FF) {
    throw std::invalid_argument("达妙电机 Master ID 必须是 11 位标准 CAN ID");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (configs_.find(config.motor_id) != configs_.end()) {
    throw std::invalid_argument("达妙电机 ID 重复：" + std::to_string(config.motor_id));
  }
  configs_.emplace(config.motor_id, config);
  states_.try_emplace(config.motor_id);
  // 4340P 不使用其他型号的固定量程兜底；收到实际量程前禁止解析和下发控制。
  limits_[config.motor_id] = {};
  position_alignments_.try_emplace(config.motor_id);
}

bool DamiaoDriver::initialize_absolute_positions(std::chrono::milliseconds timeout)
{
  std::vector<DmMotorConfig> motors;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : configs_) {
      motors.push_back(item.second);
    }
  }

  bool all_ready = true;
  for (const auto & motor : motors) {
    if (!motor.use_absolute_output_encoder) {
      continue;
    }
    const auto output_position = read_parameter(
      motor.motor_id, motor.absolute_position_register, timeout);
    if (!output_position || !std::isfinite(*output_position)) {
      all_ready = false;
      continue;
    }

    const double joint_position =
      static_cast<double>(*output_position) * motor.direction + motor.offset_rad;
    std::lock_guard<std::mutex> lock(mutex_);
    auto & alignment = position_alignments_[motor.motor_id];
    alignment.absolute_reference_valid = true;
    alignment.feedback_bias_initialized = false;
    alignment.absolute_reference_rad = joint_position;
    auto & state = states_[motor.motor_id];
    state.position_rad = joint_position;
    state.absolute_position_valid = true;
  }
  return all_ready;
}

bool DamiaoDriver::discover_limits(std::chrono::milliseconds timeout)
{
  std::vector<uint8_t> ids;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : configs_) {
      ids.push_back(item.first);
    }
  }
  bool all_ready = true;
  for (const auto id : ids) {
    const auto pmax = read_parameter(id, kPmaxRegister, timeout);
    const auto vmax = read_parameter(id, kVmaxRegister, timeout);
    const auto tmax = read_parameter(id, kTmaxRegister, timeout);
    if (pmax && vmax && tmax && std::isfinite(*pmax) && std::isfinite(*vmax) &&
      std::isfinite(*tmax) && *pmax > 0.0F && *vmax > 0.0F && *tmax > 0.0F)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      limits_[id] = {*pmax, *vmax, *tmax, true};
    } else {
      all_ready = false;
    }
  }
  return all_ready;
}

void DamiaoDriver::enable_all()
{
  std::vector<uint8_t> ids;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & item : configs_) {ids.push_back(item.first);}
  }
  for (const auto id : ids) {send_special(id, kEnable);}
}

void DamiaoDriver::disable_all() noexcept
{
  if (!transport_ || !transport_->is_open()) {return;}
  try {
    std::vector<uint8_t> ids;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & item : configs_) {ids.push_back(item.first);}
    }
    for (const auto id : ids) {send_special(id, kDisable);}
  } catch (...) {
  }
}

void DamiaoDriver::request_refresh(uint8_t motor_id)
{
  CanFrame frame;
  frame.id = 0x7FF;
  frame.size = 8;
  frame.data[0] = motor_id;
  frame.data[1] = 0x00;
  frame.data[2] = kRefresh;
  frame.data[3] = 0x00;
  transport_->send(frame);
}

void DamiaoDriver::command_position_velocity(
  uint8_t motor_id, double position_rad, double velocity_rad_s)
{
  DmMotorConfig config;
  DmMotorLimits limits;
  double feedback_bias_rad = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = configs_.find(motor_id);
    if (iterator == configs_.end()) {throw std::out_of_range("未知的达妙电机 ID");}
    config = iterator->second;
    limits = limits_.at(motor_id);
    const auto & alignment = position_alignments_.at(motor_id);
    if (alignment.feedback_bias_initialized) {
      feedback_bias_rad = alignment.feedback_bias_rad;
    }
  }
  if (config.mode != DmControlMode::kPositionVelocity) {
    throw std::logic_error("达妙电机未配置为位置速度模式");
  }
  if (!limits.read_from_drive) {
    throw std::logic_error("达妙电机量程尚未从驱动器读取，拒绝发送控制指令");
  }
  const float drive_position = static_cast<float>(
    (position_rad - feedback_bias_rad - config.offset_rad) * config.direction);
  const float drive_velocity = static_cast<float>(
    std::clamp(std::abs(velocity_rad_s), 0.0, limits.velocity_rad_s));
  CanFrame frame;
  frame.id = 0x100U + motor_id;
  frame.size = 8;
  std::memcpy(frame.data.data(), &drive_position, sizeof(float));
  std::memcpy(frame.data.data() + 4, &drive_velocity, sizeof(float));
  transport_->send(frame);
}

std::optional<DmMotorState> DamiaoDriver::state(uint8_t motor_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iterator = states_.find(motor_id);
  return iterator == states_.end() ? std::nullopt : std::optional<DmMotorState>(iterator->second);
}

std::optional<DmMotorLimits> DamiaoDriver::limits(uint8_t motor_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iterator = limits_.find(motor_id);
  return iterator == limits_.end() ? std::nullopt : std::optional<DmMotorLimits>(iterator->second);
}

void DamiaoDriver::handle_frame(const CanFrame & frame)
{
  if (frame.extended || frame.size < 8) {return;}

  if (frame.data[2] == kParameterRead) {
    const uint16_t full_motor_id = static_cast<uint16_t>(frame.data[0]) |
      (static_cast<uint16_t>(frame.data[1]) << 8U);
    if (full_motor_id <= 0xFF) {
      const uint8_t motor_id = static_cast<uint8_t>(full_motor_id);
      const uint8_t register_id = frame.data[3];
      const uint32_t key = parameter_key(motor_id, register_id);
      bool expected = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto config_iterator = configs_.find(motor_id);
        expected = config_iterator != configs_.end() &&
          frame.id == config_iterator->second.master_id &&
          pending_parameters_.find(key) != pending_parameters_.end();
        if (expected) {
          float value = 0.0F;
          std::memcpy(&value, frame.data.data() + 4, sizeof(float));
          auto & pending = pending_parameters_[key];
          pending.value = value;
          pending.ready = true;
        }
      }
      if (expected) {
        parameter_cv_.notify_all();
        return;
      }
    }
  }

  const uint8_t motor_id = frame.data[0] & 0x0F;
  DmMotorConfig config;
  DmMotorLimits limits;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto config_iterator = configs_.find(motor_id);
    if (config_iterator == configs_.end() || frame.id != config_iterator->second.master_id) {
      return;
    }
    config = config_iterator->second;
    limits = limits_[motor_id];
  }
  if (!limits.read_from_drive) {
    return;
  }

  const uint16_t position_raw = (static_cast<uint16_t>(frame.data[1]) << 8U) | frame.data[2];
  const uint16_t velocity_raw = (static_cast<uint16_t>(frame.data[3]) << 4U) | (frame.data[4] >> 4U);
  const uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8U) | frame.data[5];
  DmMotorState decoded;
  const double raw_position = uint_to_float(position_raw, -limits.position_rad, limits.position_rad, 16);
  const double mapped_position = raw_position * config.direction + config.offset_rad;
  decoded.velocity_rad_s = uint_to_float(velocity_raw, -limits.velocity_rad_s, limits.velocity_rad_s, 12) * config.direction;
  decoded.torque_nm = uint_to_float(torque_raw, -limits.torque_nm, limits.torque_nm, 12) * config.direction;
  decoded.mos_temperature_c = static_cast<double>(frame.data[6]);
  decoded.rotor_temperature_c = static_cast<double>(frame.data[7]);
  decoded.status_code = frame.data[0] >> 4U;
  decoded.online = true;
  decoded.stamp = frame.stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & alignment = position_alignments_[motor_id];
    if (alignment.absolute_reference_valid) {
      if (!alignment.feedback_bias_initialized) {
        alignment.feedback_bias_rad = std::remainder(
          alignment.absolute_reference_rad - mapped_position, kTwoPi);
        alignment.feedback_bias_initialized = true;
      }
      decoded.position_rad = mapped_position + alignment.feedback_bias_rad;
      decoded.absolute_position_valid = true;
    } else {
      decoded.position_rad = mapped_position;
    }
    states_[motor_id] = decoded;
  }
}

void DamiaoDriver::send_special(uint8_t motor_id, uint8_t command)
{
  CanFrame frame;
  // 新版协议的 FC/FD/FE 特殊命令始终发送到 ESC_ID；模式只影响运动帧 ID。
  frame.id = motor_id;
  frame.size = 8;
  std::fill_n(frame.data.begin(), 7, 0xFF);
  frame.data[7] = command;
  transport_->send(frame);
}

std::optional<float> DamiaoDriver::read_parameter(
  uint8_t motor_id, uint8_t register_id, std::chrono::milliseconds timeout)
{
  const uint32_t key = parameter_key(motor_id, register_id);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_parameters_[key] = {};
  }
  CanFrame frame;
  frame.id = 0x7FF;
  frame.size = 8;
  frame.data[0] = motor_id;
  frame.data[1] = 0x00;
  frame.data[2] = kParameterRead;
  frame.data[3] = register_id;
  transport_->send(frame);

  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = parameter_cv_.wait_for(lock, timeout, [&]() {return pending_parameters_[key].ready;});
  if (!ready) {
    pending_parameters_.erase(key);
    return std::nullopt;
  }
  const float value = pending_parameters_[key].value;
  pending_parameters_.erase(key);
  return value;
}

}  // namespace mobile_base
