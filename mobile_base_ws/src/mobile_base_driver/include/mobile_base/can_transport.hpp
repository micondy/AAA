#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace mobile_base
{

struct CanFrame
{
  uint32_t id{0};
  bool extended{false};
  bool fd{false};
  uint8_t size{0};
  std::array<uint8_t, 64> data{};
  std::chrono::steady_clock::time_point stamp{};
};

class CanTransport
{
public:
  using ReceiveCallback = std::function<void(const CanFrame &)>;
  virtual ~CanTransport() = default;
  virtual void open(const std::string & interface_name, bool enable_can_fd) = 0;
  virtual void close() noexcept = 0;
  virtual bool is_open() const noexcept = 0;
  virtual void set_receive_callback(ReceiveCallback callback) = 0;
  virtual void send(const CanFrame & frame) = 0;
};

std::unique_ptr<CanTransport> make_socketcan_transport();

}  // namespace mobile_base
