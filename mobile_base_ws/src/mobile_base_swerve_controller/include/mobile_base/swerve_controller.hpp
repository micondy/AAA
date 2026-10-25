#pragma once
#include "mobile_base/bounded_motion.hpp"

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace mobile_base
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Module
  {
    std::string name;
    std::string steering_joint;
    std::string drive_joint;
    double x{0.0};
    double y{0.0};
    size_t steering_command{0};
    size_t drive_command{0};
    size_t steering_state{0};
    size_t drive_state{0};
    double held_angle{0.0};
  };

  struct Command
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    std::chrono::steady_clock::time_point stamp{};
  };

  template<typename InterfaceContainer>
  static size_t find_interface(const InterfaceContainer & interfaces, const std::string & full_name);
  void update_odometry(const rclcpp::Time & time, double dt);

  std::array<Module, 4> modules_{};
  double wheel_radius_{0.10};
  double max_wheel_angular_velocity_{20.0};
  double max_wheel_angular_acceleration_{8.0};
  double steering_limit_rad_{1.5707963267948966};
  BoundedMotion motion_;
  double drive_alignment_tolerance_rad_{0.0872664625997165};
  double command_timeout_s_{0.25};
  bool publish_odometry_{true};
  bool publish_odom_tf_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
};

}  // namespace mobile_base
