#include "mobile_base/swerve_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace mobile_base
{
namespace
{
const auto kLogger = rclcpp::get_logger("mobile_base.hardware");
rclcpp::Clock kThrottleClock(RCL_STEADY_TIME);

std::string parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto iterator = parameters.find(name);
  return iterator == parameters.end() ? fallback : iterator->second;
}

double double_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, double fallback)
{
  return std::stod(parameter_or(parameters, name, std::to_string(fallback)));
}

int int_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, int fallback)
{
  return std::stoi(parameter_or(parameters, name, std::to_string(fallback)), nullptr, 0);
}

bool bool_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, bool fallback)
{
  const auto value = parameter_or(parameters, name, fallback ? "true" : "false");
  return value == "true" || value == "True" || value == "1";
}
}  // namespace

hardware_interface::CallbackReturn SwerveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    dm_interface_ = parameter_or(info_.hardware_parameters, "dm_can_interface", "can0");
    zlac_interface_ = parameter_or(info_.hardware_parameters, "zlac_can_interface", "can1");
    dm_can_fd_ = bool_parameter(info_.hardware_parameters, "dm_can_fd", false);
    require_absolute_steering_ = bool_parameter(
      info_.hardware_parameters, "require_absolute_steering", true);
    transaction_timeout_ = std::chrono::milliseconds(
      int_parameter(info_.hardware_parameters, "transaction_timeout_ms", 100));
    state_timeout_ = std::chrono::milliseconds(
      int_parameter(info_.hardware_parameters, "state_timeout_ms", 250));
    drive_acceleration_ = int_parameter(info_.hardware_parameters, "drive_acceleration", 100);
    drive_deceleration_ = int_parameter(info_.hardware_parameters, "drive_deceleration", 100);
    drive_communication_watchdog_ms_ = int_parameter(
      info_.hardware_parameters, "drive_communication_watchdog_ms", 500);
    drive_diagnostic_period_ = std::chrono::milliseconds(
      int_parameter(info_.hardware_parameters, "drive_diagnostic_period_ms", 100));
    if (drive_diagnostic_period_.count() <= 0) {
      throw std::invalid_argument("drive_diagnostic_period_ms 必须大于 0");
    }

    const size_t count = info_.joints.size();
    bindings_.reserve(count);
    positions_.assign(count, 0.0);
    velocities_.assign(count, 0.0);
    efforts_.assign(count, 0.0);
    commands_.assign(count, 0.0);
    steering_command_velocity_.assign(count, 0.5);

    dm_ = std::make_unique<DamiaoDriver>(make_socketcan_transport());
    zlac_ = std::make_unique<Zlac8015dDriver>(make_socketcan_transport());

    ZlacObjectDictionary object_dictionary;
    object_dictionary.communication_watchdog = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_communication_watchdog_index", 0x2000));
    object_dictionary.synchronous_control = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_synchronous_control_index", 0x200F));
    object_dictionary.controlword = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_controlword_index", 0x6040));
    object_dictionary.fault_code = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_fault_code_index", 0x603F));
    object_dictionary.statusword = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_statusword_index", 0x6041));
    object_dictionary.mode_of_operation = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_mode_index", 0x6060));
    object_dictionary.mode_display = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_mode_display_index", 0x6061));
    object_dictionary.target_velocity = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_target_velocity_index", 0x60FF));
    object_dictionary.actual_velocity = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_actual_velocity_index", 0x606C));
    object_dictionary.acceleration = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_acceleration_index", 0x6083));
    object_dictionary.deceleration = static_cast<uint16_t>(
      int_parameter(info_.hardware_parameters, "zlac_deceleration_index", 0x6084));
    zlac_->set_object_dictionary(object_dictionary);

    for (size_t index = 0; index < count; ++index) {
      const auto & joint = info_.joints[index];
      const auto kind = parameter_or(joint.parameters, "kind", "");
      JointBinding binding;
      binding.name = joint.name;
      if (kind == "steering") {
        if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          throw std::invalid_argument(joint.name + " 必须提供一个 position 指令接口");
        }
        binding.kind = JointKind::kSteering;
        binding.motor_id = static_cast<uint8_t>(int_parameter(joint.parameters, "motor_id", 0));
        steering_command_velocity_[index] = double_parameter(
          joint.parameters, "command_velocity_rad_s", 0.5);
        if (!std::isfinite(steering_command_velocity_[index]) ||
          steering_command_velocity_[index] <= 0.0)
        {
          throw std::invalid_argument(joint.name + " 的转向速度上限必须大于 0");
        }
        DmMotorConfig motor;
        motor.joint_name = joint.name;
        motor.model_name = parameter_or(
          joint.parameters, "motor_model", "DM-J4340P-2EC-24V");
        motor.motor_id = binding.motor_id;
        motor.master_id = static_cast<uint16_t>(int_parameter(joint.parameters, "master_id", 0));
        motor.mode = DmControlMode::kPositionVelocity;
        motor.direction = double_parameter(joint.parameters, "direction", 1.0);
        motor.offset_rad = double_parameter(joint.parameters, "offset_rad", 0.0);
        motor.command_velocity_rad_s = steering_command_velocity_[index];
        motor.use_absolute_output_encoder = bool_parameter(
          joint.parameters, "use_absolute_output_encoder", true);
        motor.absolute_position_register = static_cast<uint8_t>(int_parameter(
          joint.parameters, "absolute_position_register", 0x51));
        if (require_absolute_steering_ && !motor.use_absolute_output_encoder) {
          throw std::invalid_argument(
            joint.name + " 在 require_absolute_steering=true 时必须启用输出轴绝对编码器");
        }
        dm_->add_motor(motor);
      } else if (kind == "drive") {
        if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          throw std::invalid_argument(joint.name + " 必须提供一个 velocity 指令接口");
        }
        binding.kind = JointKind::kDrive;
        binding.node_id = static_cast<uint8_t>(int_parameter(joint.parameters, "node_id", 0));
        binding.channel = static_cast<uint8_t>(int_parameter(joint.parameters, "channel", 0));
        ZlacChannelConfig channel;
        channel.joint_name = joint.name;
        channel.node_id = binding.node_id;
        channel.channel = binding.channel;
        channel.direction = double_parameter(joint.parameters, "direction", 1.0);
        channel.gear_ratio = double_parameter(joint.parameters, "gear_ratio", 1.0);
        channel.max_motor_rpm = double_parameter(joint.parameters, "max_motor_rpm", 1000.0);
        channel.command_units_per_rpm = double_parameter(
          joint.parameters, "command_units_per_rpm", 1.0);
        channel.feedback_rpm_per_unit = double_parameter(
          joint.parameters, "feedback_rpm_per_unit", 0.1);
        zlac_->add_channel(channel);
      } else {
        throw std::invalid_argument(joint.name + " 缺少有效的 kind=steering|drive 参数");
      }
      bindings_.push_back(binding);
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(kLogger, "硬件配置无效：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    dm_->open(dm_interface_, dm_can_fd_);
    zlac_->open(zlac_interface_);
    if (!dm_->discover_limits(transaction_timeout_)) {
      throw std::runtime_error(
              "至少一台 DM-J4340P-2EC 24V 无法读取 PMAX/VMAX/TMAX，已拒绝使能");
    }
    if (!dm_->initialize_absolute_positions(transaction_timeout_) &&
      require_absolute_steering_)
    {
      throw std::runtime_error(
        "至少一台 DM-J4340P-2EC 24V 无法读取输出轴绝对位置寄存器 0x51");
    }
    zlac_->configure_profile_velocity(
      drive_acceleration_, drive_deceleration_, drive_communication_watchdog_ms_,
      transaction_timeout_);
    for (size_t index = 0; index < bindings_.size(); ++index) {
      const auto & binding = bindings_[index];
      if (binding.kind == JointKind::kSteering) {
        const auto state = dm_->state(binding.motor_id);
        if (state && state->absolute_position_valid) {
          positions_[index] = state->position_rad;
          commands_[index] = state->position_rad;
          RCLCPP_INFO(kLogger, "%s 输出轴绝对位置：%.6f rad",
            binding.name.c_str(), state->position_rad);
        }
        dm_->request_refresh(binding.motor_id);
      } else {
        zlac_->request_velocity(binding.node_id, binding.channel);
      }
    }
  } catch (const std::exception & exception) {
    zlac_->disable_all();
    dm_->disable_all();
    zlac_->close();
    dm_->close();
    RCLCPP_ERROR(kLogger, "配置 CAN 硬件失败：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  try {
    for (size_t index = 0; index < bindings_.size(); ++index) {
      commands_[index] = bindings_[index].kind == JointKind::kSteering ? positions_[index] : 0.0;
    }
    dm_->enable_all();
    zlac_->enable_all(transaction_timeout_);
    active_started_ = std::chrono::steady_clock::now();
    last_drive_diagnostic_poll_ = {};
    active_ = true;
    RCLCPP_INFO(kLogger, "DM-J4340P-2EC 24V 量程和输出轴绝对位置已载入，底盘硬件已使能");
  } catch (const std::exception & exception) {
    zlac_->disable_all();
    dm_->disable_all();
    RCLCPP_ERROR(kLogger, "底盘使能失败：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  for (const auto & binding : bindings_) {
    if (binding.kind == JointKind::kDrive) {
      try {zlac_->command_velocity(binding.node_id, binding.channel, 0.0);} catch (...) {}
    }
  }
  zlac_->disable_all();
  dm_->disable_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SwerveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t index = 0; index < bindings_.size(); ++index) {
    interfaces.emplace_back(bindings_[index].name, hardware_interface::HW_IF_POSITION, &positions_[index]);
    interfaces.emplace_back(bindings_[index].name, hardware_interface::HW_IF_VELOCITY, &velocities_[index]);
    interfaces.emplace_back(bindings_[index].name, hardware_interface::HW_IF_EFFORT, &efforts_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SwerveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t index = 0; index < bindings_.size(); ++index) {
    interfaces.emplace_back(
      bindings_[index].name,
      bindings_[index].kind == JointKind::kSteering ?
      hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY,
      &commands_[index]);
  }
  return interfaces;
}

hardware_interface::return_type SwerveHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const auto now = std::chrono::steady_clock::now();
  const bool poll_drive_diagnostics = last_drive_diagnostic_poll_.time_since_epoch().count() == 0 ||
    now - last_drive_diagnostic_poll_ >= drive_diagnostic_period_;
  std::unordered_set<uint8_t> diagnostic_nodes;
  try {
    for (size_t index = 0; index < bindings_.size(); ++index) {
      const auto & binding = bindings_[index];
      if (binding.kind == JointKind::kSteering) {
        dm_->request_refresh(binding.motor_id);
        const auto state = dm_->state(binding.motor_id);
        if (state && state->online) {
          positions_[index] = state->position_rad;
          velocities_[index] = state->velocity_rad_s;
          efforts_[index] = state->torque_nm;
          const bool motor_fault = state->status_code != 0 && state->status_code != 1;
          const bool not_enabled = state->status_code == 0 &&
            now - active_started_ > state_timeout_;
          if (active_ && (motor_fault || not_enabled ||
            now - state->stamp > state_timeout_))
          {
            RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
              "DM-J4340P-2EC 24V ID %u 未使能、发生故障或反馈超时（状态码=%u）",
              binding.motor_id, state->status_code);
            return hardware_interface::return_type::ERROR;
          }
        } else if (active_ && now - active_started_ > state_timeout_) {
          RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
            "未收到 DM-J4340P-2EC 24V ID %u 的反馈", binding.motor_id);
          return hardware_interface::return_type::ERROR;
        }
      } else {
        if (poll_drive_diagnostics && diagnostic_nodes.insert(binding.node_id).second) {
          zlac_->request_diagnostics(binding.node_id);
        }
        zlac_->request_velocity(binding.node_id, binding.channel);
        const auto state = zlac_->state(binding.node_id, binding.channel);
        if (state && state->online) {
          velocities_[index] = state->velocity_rad_s;
          positions_[index] += velocities_[index] * period.seconds();
          efforts_[index] = std::numeric_limits<double>::quiet_NaN();
          const bool alarm = state->diagnostics_valid && (state->statusword & 0x0080U) != 0;
          const bool external_emergency =
            state->diagnostics_valid && (state->statusword & 0x8000U) != 0;
          const bool not_enabled = state->diagnostics_valid &&
            (state->statusword & 0x006FU) != 0x0027U;
          if (active_ && (alarm || external_emergency || not_enabled)) {
            RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
              "ZLAC8015D 节点 %u 通道 %u 状态异常（状态字=0x%04X，故障码=0x%08X）",
              binding.node_id, binding.channel,
              static_cast<unsigned int>(state->statusword),
              static_cast<unsigned int>(state->fault_code));
            return hardware_interface::return_type::ERROR;
          }
          if (active_ && now - state->stamp > state_timeout_) {
            RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
              "ZLAC8015D 节点 %u 通道 %u 反馈超时", binding.node_id, binding.channel);
            return hardware_interface::return_type::ERROR;
          }
        } else if (active_ && now - active_started_ > state_timeout_) {
          RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
            "未收到 ZLAC8015D 节点 %u 通道 %u 的反馈",
            binding.node_id, binding.channel);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    if (poll_drive_diagnostics) {last_drive_diagnostic_poll_ = now;}
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
      "CAN 读取失败：%s", exception.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SwerveHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  try {
    for (size_t index = 0; index < bindings_.size(); ++index) {
      const auto & binding = bindings_[index];
      if (!std::isfinite(commands_[index])) {continue;}
      if (binding.kind == JointKind::kSteering) {
        dm_->command_position_velocity(
          binding.motor_id, commands_[index], steering_command_velocity_[index]);
      } else {
        zlac_->command_velocity(binding.node_id, binding.channel, commands_[index]);
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
      "CAN 写入失败：%s", exception.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mobile_base

PLUGINLIB_EXPORT_CLASS(mobile_base::SwerveHardware, hardware_interface::SystemInterface)
