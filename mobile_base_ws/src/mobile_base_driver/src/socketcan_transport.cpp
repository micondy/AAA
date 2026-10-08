#include "mobile_base/can_transport.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef __linux__
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace mobile_base
{

class SocketCanTransport final : public CanTransport
{
public:
  ~SocketCanTransport() override {close();}

  void open(const std::string & interface_name, bool enable_can_fd) override
  {
#ifdef __linux__
    close();
    const int socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
      throw std::runtime_error("创建 SocketCAN 套接字失败：" + std::string(std::strerror(errno)));
    }

    if (enable_can_fd) {
      const int enabled = 1;
      if (::setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)) < 0) {
        ::close(socket_fd);
        throw std::runtime_error("该 SocketCAN 接口不支持 CAN-FD 帧");
      }
    }

    struct ifreq request {};
    if (interface_name.size() >= IFNAMSIZ) {
      ::close(socket_fd);
      throw std::invalid_argument("SocketCAN 接口名称过长：" + interface_name);
    }
    std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_fd, SIOCGIFINDEX, &request) < 0) {
      ::close(socket_fd);
      throw std::runtime_error("找不到 SocketCAN 接口：" + interface_name);
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket_fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      ::close(socket_fd);
      throw std::runtime_error("绑定 SocketCAN 接口失败：" + interface_name);
    }

    socket_fd_ = socket_fd;
    can_fd_ = enable_can_fd;
    running_.store(true);
    receive_thread_ = std::thread([this]() {receive_loop();});
#else
    (void)interface_name;
    (void)enable_can_fd;
    throw std::runtime_error("SocketCAN 仅支持 Linux；请将工作区迁移到 Linux 后再使用");
#endif
  }

  void close() noexcept override
  {
#ifdef __linux__
    running_.store(false);
    if (socket_fd_ >= 0) {
      ::shutdown(socket_fd_, SHUT_RDWR);
    }
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
#endif
  }

  bool is_open() const noexcept override
  {
#ifdef __linux__
    return socket_fd_ >= 0 && running_.load();
#else
    return false;
#endif
  }

  void set_receive_callback(ReceiveCallback callback) override
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
  }

  void send(const CanFrame & frame) override
  {
#ifdef __linux__
    if (!is_open()) {
      throw std::runtime_error("试图通过未打开的 SocketCAN 接口发送报文");
    }
    const canid_t id = static_cast<canid_t>(frame.id) |
      (frame.extended ? CAN_EFF_FLAG : 0U);
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (frame.fd) {
      if (!can_fd_ || frame.size > CANFD_MAX_DLEN) {
        throw std::invalid_argument("CAN-FD 报文无效");
      }
      struct canfd_frame native {};
      native.can_id = id;
      native.len = frame.size;
      std::copy_n(frame.data.begin(), frame.size, native.data);
      if (::write(socket_fd_, &native, CANFD_MTU) != CANFD_MTU) {
        throw std::runtime_error("发送 CAN-FD 报文失败");
      }
    } else {
      if (frame.size > CAN_MAX_DLEN) {
        throw std::invalid_argument("经典 CAN 报文长度超过 8 字节");
      }
      struct can_frame native {};
      native.can_id = id;
      native.can_dlc = frame.size;
      std::copy_n(frame.data.begin(), frame.size, native.data);
      if (::write(socket_fd_, &native, CAN_MTU) != CAN_MTU) {
        throw std::runtime_error("发送经典 CAN 报文失败");
      }
    }
#else
    (void)frame;
    throw std::runtime_error("Windows 下无法使用 SocketCAN 发送报文");
#endif
  }

private:
#ifdef __linux__
  void receive_loop() noexcept
  {
    while (running_.load()) {
      struct pollfd descriptor {socket_fd_, POLLIN, 0};
      const int result = ::poll(&descriptor, 1, 100);
      if (result <= 0 || !(descriptor.revents & POLLIN)) {
        continue;
      }

      struct canfd_frame native {};
      const ssize_t bytes = ::read(socket_fd_, &native, CANFD_MTU);
      if (bytes != CAN_MTU && bytes != CANFD_MTU) {
        continue;
      }

      CanFrame frame;
      frame.extended = (native.can_id & CAN_EFF_FLAG) != 0;
      frame.id = native.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      frame.fd = bytes == CANFD_MTU;
      frame.size = frame.fd ? native.len : reinterpret_cast<struct can_frame *>(&native)->can_dlc;
      std::copy_n(native.data, frame.size, frame.data.begin());
      frame.stamp = std::chrono::steady_clock::now();

      ReceiveCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = callback_;
      }
      if (callback) {
        callback(frame);
      }
    }
  }

  int socket_fd_{-1};
  bool can_fd_{false};
  std::atomic_bool running_{false};
  std::thread receive_thread_;
  std::mutex write_mutex_;
#endif
  std::mutex callback_mutex_;
  ReceiveCallback callback_;
};

std::unique_ptr<CanTransport> make_socketcan_transport()
{
  return std::make_unique<SocketCanTransport>();
}

}  // namespace mobile_base
