#include "mobile_base/swerve_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_base
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
const std::array<std::string, 4> kModuleNames{
  "front_left", "front_right", "rear_left", "rear_right"};
}  // namespace

controller_interface::CallbackReturn SwerveController::on_init()
{
  try {
    auto_declare<double>("wheel_radius", 0.10);
    auto_declare<double>("max_wheel_angular_velocity", 20.0);
    auto_declare<double>("max_wheel_angular_acceleration", 8.0);
    auto_declare<double>("steering_limit_rad", kPi / 2.0);
    auto_declare<double>("max_steering_rate_rad_s", 0.15);
    auto_declare<double>("drive_alignment_tolerance_rad", 5.0 * kPi / 180.0);
    auto_declare<double>("command_timeout", 0.25);
    auto_declare<bool>("publish_odometry", true);
    auto_declare<bool>("publish_odom_tf", true);
    auto_declare<std::string>("odom_frame", "odom");
    auto_declare<std::string>("base_frame", "base_link");
    for (size_t index = 0; index < modules_.size(); ++index) {
      auto & module = modules_[index];
      module.name = kModuleNames[index];
      const std::string prefix = module.name + ".";
      auto_declare<std::string>(prefix + "steering_joint", module.name + "_steer_joint");
      auto_declare<std::string>(prefix + "drive_joint", module.name + "_drive_joint");
      auto_declare<double>(prefix + "x", index < 2 ? 0.25 : -0.25);
      auto_declare<double>(prefix + "y", (index == 0 || index == 2) ? 0.20 : -0.20);
    }
    command_buffer_.writeFromNonRT(Command{});
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "控制器参数声明失败：%s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & module : modules_) {
    configuration.names.push_back(module.steering_joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(module.drive_joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & module : modules_) {
    configuration.names.push_back(module.steering_joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(module.drive_joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn SwerveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    max_wheel_angular_velocity_ =
      get_node()->get_parameter("max_wheel_angular_velocity").as_double();
    max_wheel_angular_acceleration_ =
      get_node()->get_parameter("max_wheel_angular_acceleration").as_double();
    steering_limit_rad_ = get_node()->get_parameter("steering_limit_rad").as_double();
    motion_.steering_rate = get_node()->get_parameter("max_steering_rate_rad_s").as_double();
    drive_alignment_tolerance_rad_ =
      get_node()->get_parameter("drive_alignment_tolerance_rad").as_double();
    command_timeout_s_ = get_node()->get_parameter("command_timeout").as_double();
    publish_odometry_ = get_node()->get_parameter("publish_odometry").as_bool();
    publish_odom_tf_ = get_node()->get_parameter("publish_odom_tf").as_bool();
    odom_frame_ = get_node()->get_parameter("odom_frame").as_string();
    base_frame_ = get_node()->get_parameter("base_frame").as_string();
    if (wheel_radius_ <= 0.0 || max_wheel_angular_velocity_ <= 0.0 ||
      max_wheel_angular_acceleration_ <= 0.0 || !std::isfinite(motion_.steering_rate) ||
      motion_.steering_rate <= 0.0 ||
      command_timeout_s_ <= 0.0 || steering_limit_rad_ <= 0.0 ||
      steering_limit_rad_ > kPi / 2.0 || drive_alignment_tolerance_rad_ <= 0.0 ||
      drive_alignment_tolerance_rad_ > kPi / 2.0)
    {
      throw std::invalid_argument("轮胎半径、轮速上限和指令超时时间必须为正数");
    }
    for (auto & module : modules_) {
      const std::string prefix = module.name + ".";
      module.steering_joint = get_node()->get_parameter(prefix + "steering_joint").as_string();
      module.drive_joint = get_node()->get_parameter(prefix + "drive_joint").as_string();
      module.x = get_node()->get_parameter(prefix + "x").as_double();
      module.y = get_node()->get_parameter(prefix + "y").as_double();
    }

    motion_.radius = wheel_radius_;
    motion_.limit = steering_limit_rad_;
    motion_.alignment = drive_alignment_tolerance_rad_;
    motion_.acceleration = max_wheel_angular_acceleration_;
    motion_.max_speed = max_wheel_angular_velocity_;
    for (size_t i = 0; i < modules_.size(); ++i) {
      motion_.x[i] = modules_[i].x;
      motion_.y[i] = modules_[i].y;
    }

    command_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        Command command;
        command.vx = message->linear.x;
        command.vy = message->linear.y;
        command.wz = message->angular.z;
        command.stamp = std::chrono::steady_clock::now();
        command_buffer_.writeFromNonRT(command);
      });
    if (publish_odometry_) {
      odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        "~/odom", rclcpp::SystemDefaultsQoS());
      if (publish_odom_tf_) {
        odom_tf_broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "控制器配置失败：%s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  try {
    for (auto & module : modules_) {
      module.steering_command = find_interface(
        command_interfaces_, module.steering_joint + "/" + hardware_interface::HW_IF_POSITION);
      module.drive_command = find_interface(
        command_interfaces_, module.drive_joint + "/" + hardware_interface::HW_IF_VELOCITY);
      module.steering_state = find_interface(
        state_interfaces_, module.steering_joint + "/" + hardware_interface::HW_IF_POSITION);
      module.drive_state = find_interface(
        state_interfaces_, module.drive_joint + "/" + hardware_interface::HW_IF_VELOCITY);
      module.held_angle = state_interfaces_[module.steering_state].get_value();
      if (!std::isfinite(module.held_angle) ||
        std::abs(module.held_angle) > steering_limit_rad_ + 0.01) {
        throw std::runtime_error("转向反馈超出限位，拒绝激活");
      }
      command_interfaces_[module.steering_command].set_value(module.held_angle);
      command_interfaces_[module.drive_command].set_value(0.0);
    }
    BoundedMotion::Four actual{};
    for (size_t i = 0; i < modules_.size(); ++i) {actual[i] = modules_[i].held_angle;}
    if (!motion_.reset(actual)) {throw std::runtime_error("转向初始位置无效");}
    Command stopped;
    stopped.stamp = std::chrono::steady_clock::now();
    command_buffer_.writeFromNonRT(stopped);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "控制器接口绑定失败：%s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & module : modules_) {
    command_interfaces_[module.drive_command].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const Command * stored = command_buffer_.readFromRT();
  Command command = stored ? *stored : Command{};
  if (command.stamp == std::chrono::steady_clock::time_point{} ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - command.stamp).count() >
    command_timeout_s_)
  {
    command.vx = command.vy = command.wz = 0.0;
  }

  BoundedMotion::Four actual{}, velocity{};
  for (size_t i = 0; i < modules_.size(); ++i) {
    actual[i] = state_interfaces_[modules_[i].steering_state].get_value();
    velocity[i] = state_interfaces_[modules_[i].drive_state].get_value();
  }
  const bool valid = motion_.step(
    command.vx, command.vy, command.wz, actual, velocity, period.seconds());
  for (size_t i = 0; i < modules_.size(); ++i) {
    (void)command_interfaces_[modules_[i].steering_command].set_value(motion_.angle[i]);
    (void)command_interfaces_[modules_[i].drive_command].set_value(motion_.speed[i]);
  }
  if (!valid) {
    RCLCPP_ERROR(get_node()->get_logger(), "转向反馈越限、非有限值或命令无效，已停止行走");
    return controller_interface::return_type::ERROR;
  }

  if (publish_odometry_) {update_odometry(time, period.seconds());}
  return controller_interface::return_type::OK;
}

template<typename InterfaceContainer>
size_t SwerveController::find_interface(
  const InterfaceContainer & interfaces, const std::string & full_name)
{
  for (size_t index = 0; index < interfaces.size(); ++index) {
    if (interfaces[index].get_name() == full_name) {return index;}
  }
  throw std::runtime_error("缺少所需的 ros2_control 接口：" + full_name);
}

void SwerveController::update_odometry(const rclcpp::Time & time, double dt)
{
  double vx = 0.0;
  double vy = 0.0;
  double rotation_numerator = 0.0;
  double rotation_denominator = 0.0;
  for (const auto & module : modules_) {
    const double angle = state_interfaces_[module.steering_state].get_value();
    const double linear_speed =
      state_interfaces_[module.drive_state].get_value() * wheel_radius_;
    const double module_vx = linear_speed * std::cos(angle);
    const double module_vy = linear_speed * std::sin(angle);
    vx += module_vx;
    vy += module_vy;
    rotation_numerator += -module.y * module_vx + module.x * module_vy;
    rotation_denominator += module.x * module.x + module.y * module.y;
  }
  vx /= static_cast<double>(modules_.size());
  vy /= static_cast<double>(modules_.size());
  const double wz = rotation_denominator > std::numeric_limits<double>::epsilon() ?
    rotation_numerator / rotation_denominator : 0.0;

  odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
  odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
  odom_yaw_ += wz * dt;

  nav_msgs::msg::Odometry message;
  message.header.stamp = time;
  message.header.frame_id = odom_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = odom_x_;
  message.pose.pose.position.y = odom_y_;
  message.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
  message.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
  message.twist.twist.linear.x = vx;
  message.twist.twist.linear.y = vy;
  message.twist.twist.angular.z = wz;
  odometry_publisher_->publish(message);
  if (odom_tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = message.child_frame_id;
    transform.transform.translation.x = message.pose.pose.position.x;
    transform.transform.translation.y = message.pose.pose.position.y;
    transform.transform.translation.z = message.pose.pose.position.z;
    transform.transform.rotation = message.pose.pose.orientation;
    odom_tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace mobile_base

PLUGINLIB_EXPORT_CLASS(mobile_base::SwerveController, controller_interface::ControllerInterface)
