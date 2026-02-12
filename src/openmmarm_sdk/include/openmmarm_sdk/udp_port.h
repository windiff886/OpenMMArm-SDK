#pragma once

/**
 * @file udp_port.h
 * @brief UDP 通信端口封装
 *
 * 提供可靠的 UDP 通信，支持 CRC32 校验、非阻塞接收、
 * 自动重连检测等功能。可作为 Client 或 Server 使用。
 */

#include <arpa/inet.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "openmmarm_sdk/timer.h"

namespace OPENMMARM_SDK {

/// 通信状态枚举
enum class IOPortStatus { OK, ERROR, TIMEOUT, MSG_BAD, CRC_BAD };

inline std::ostream &operator<<(std::ostream &os, const IOPortStatus &status) {
  switch (status) {
  case IOPortStatus::OK:
    os << "OK";
    break;
  case IOPortStatus::ERROR:
    os << "ERROR";
    break;
  case IOPortStatus::TIMEOUT:
    os << "TIMEOUT";
    break;
  case IOPortStatus::MSG_BAD:
    os << "MSG_BAD";
    break;
  case IOPortStatus::CRC_BAD:
    os << "CRC_BAD";
    break;
  }
  return os;
}

/**
 * @brief UDP 通信端口
 *
 * 支持两种模式：
 * - Server 模式：绑定本地端口，等待对端发来的第一个包后自动设定目标地址
 * - Client 模式：指定目标 IP 和端口，直接通信
 */
class UdpPort {
public:
  /**
   * @brief 构造 Server 模式端口
   * @param name       端口名（日志标识）
   * @param ownPort    绑定的本地端口
   * @param timeout_ms 单次接收超时（毫秒）
   */
  UdpPort(std::string name, unsigned int ownPort, size_t timeout_ms = 1000);

  /**
   * @brief 构造 Client 模式端口
   * @param name       端口名（日志标识）
   * @param ownPort    绑定的本地端口（0 则由系统分配）
   * @param toIP       目标 IP 地址
   * @param toPort     目标端口
   * @param timeout_ms 单次接收超时（毫秒）
   */
  UdpPort(std::string name, unsigned int ownPort, std::string toIP,
          unsigned int toPort, size_t timeout_ms = 1000);

  ~UdpPort() = default;

  /// 发送数据（若开启 CRC，自动附加校验码到末尾）
  IOPortStatus send(uint8_t *sendMsg, size_t sendLength);

  /// 接收数据（非阻塞，含长度和 CRC 校验）
  IOPortStatus recv(uint8_t *recvMsg, size_t recvLength);

  /// 接收不定长数据（不进行校验）
  size_t recvLen(uint8_t *recvMsg, size_t maxLength);

  /// 是否与对端保持连接
  bool isConnected() const { return is_connected_; }

  /// 是否开启 CRC32 校验
  void setCRC32(bool enable) { crc32_enabled_ = enable; }

private:
  /// CRC32 计算（与 z1_sdk / openmmarm_hw 中使用的算法一致）
  static uint32_t crc32(const void *data, size_t length);

  /// 检查接收数据的 CRC32
  bool checkCRC32(uint8_t *data, size_t len);

  /// 在数据末尾添加 CRC32
  void addCRC32(uint8_t *data, size_t len);

  /// 验证 IPv4 地址格式
  static bool isValidIPv4(const std::string &ip);

  // Socket 相关
  int socket_fd_ = -1;
  sockaddr_in own_addr_{};
  sockaddr_in target_addr_{};
  sockaddr_in from_addr_{};
  socklen_t sockaddr_size_ = sizeof(struct sockaddr);
  int reuse_on_ = 1;

  // 状态
  std::string name_;
  Timer timer_;
  bool is_server_ = false;
  bool crc32_enabled_ = false;
  bool is_connected_ = false;
  timeval timeout_saved_{};

  // 缓冲区
  std::array<uint8_t, 1500> recv_buffer_{};
};

using UdpPortPtr = std::shared_ptr<UdpPort>;

} // namespace OPENMMARM_SDK
