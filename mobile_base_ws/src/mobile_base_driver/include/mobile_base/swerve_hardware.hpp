#pragma once

#include "mobile_base/damiao_driver.hpp"
#include "mobile_base/zlac8015d_driver.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mobile_base
{

class SwerveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SwerveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class JointKind { kSteering, kDrive };
  struct JointBinding
  {
    std::string name;
    JointKind kind{JointKind::kDrive};
    uint8_t motor_id{0};
    uint8_t node_id{0};
    uint8_t channel{0};
  };

  std::string dm_interface_{"can0"};
  std::string zlac_interface_{"can1"};
  bool dm_can_fd_{false};
  bool require_absolute_steering_{true};
  std::chrono::milliseconds transaction_timeout_{100};
  std::chrono::milliseconds state_timeout_{250};
  int32_t drive_acceleration_{100};
  int32_t drive_deceleration_{100};
  int32_t drive_communication_watchdog_ms_{500};
  std::chrono::milliseconds drive_diagnostic_period_{100};

  std::unique_ptr<DamiaoDriver> dm_;
  std::unique_ptr<Zlac8015dDriver> zlac_;
  std::vector<JointBinding> bindings_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> commands_;
  std::vector<double> steering_command_velocity_;
  std::chrono::steady_clock::time_point active_started_{};
  std::chrono::steady_clock::time_point last_drive_diagnostic_poll_{};
  bool active_{false};
};

}  // namespace mobile_base
