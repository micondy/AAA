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
#include <vector>

namespace mobile_base
{

enum class DmControlMode : uint8_t { kMit, kPositionVelocity, kVelocity };

struct DmMotorConfig
{
  std::string joint_name;
  std::string model_name{"DM-J4340P-2EC-24V"};
  uint8_t motor_id{1};
  uint16_t master_id{0};
  DmControlMode mode{DmControlMode::kPositionVelocity};
  double direction{1.0};
  double offset_rad{0.0};
  double command_velocity_rad_s{0.5};
  bool use_absolute_output_encoder{true};
  uint8_t absolute_position_register{0x51};
};

struct DmMotorState
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  double mos_temperature_c{0.0};
  double rotor_temperature_c{0.0};
  uint8_t status_code{0};
  bool absolute_position_valid{false};
  bool online{false};
  std::chrono::steady_clock::time_point stamp{};
};

struct DmMotorLimits
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  bool read_from_drive{false};
};

class DamiaoDriver
{
public:
  explicit DamiaoDriver(std::unique_ptr<CanTransport> transport);
  ~DamiaoDriver();

  void open(const std::string & interface_name, bool can_fd = false);
  void close() noexcept;
  void add_motor(const DmMotorConfig & config);
  bool discover_limits(std::chrono::milliseconds timeout);
  bool initialize_absolute_positions(std::chrono::milliseconds timeout);
  void enable_all();
  void disable_all() noexcept;
  void request_refresh(uint8_t motor_id);
  void command_position_velocity(uint8_t motor_id, double position_rad, double velocity_rad_s);
  std::optional<DmMotorState> state(uint8_t motor_id) const;
  std::optional<DmMotorLimits> limits(uint8_t motor_id) const;

private:
  struct PendingParameter
  {
    bool ready{false};
    float value{0.0F};
  };

  struct PositionAlignment
  {
    bool absolute_reference_valid{false};
    bool feedback_bias_initialized{false};
    double absolute_reference_rad{0.0};
    double feedback_bias_rad{0.0};
  };

  void handle_frame(const CanFrame & frame);
  void send_special(uint8_t motor_id, uint8_t command);
  std::optional<float> read_parameter(
    uint8_t motor_id, uint8_t register_id, std::chrono::milliseconds timeout);

  std::unique_ptr<CanTransport> transport_;
  std::unordered_map<uint8_t, DmMotorConfig> configs_;
  mutable std::mutex mutex_;
  std::condition_variable parameter_cv_;
  std::unordered_map<uint8_t, DmMotorState> states_;
  std::unordered_map<uint8_t, DmMotorLimits> limits_;
  std::unordered_map<uint8_t, PositionAlignment> position_alignments_;
  std::unordered_map<uint32_t, PendingParameter> pending_parameters_;
};

}  // namespace mobile_base
