/**
 * @file udp_port.cpp
 * @brief UdpPort 实现
 *
 * 基于 z1_sdk 的 UdpPort 移植，去除 Boost 依赖，使用自实现 CRC32。
 */

#include "openmmarm_sdk/udp_port.h"

#include <cstring>
#include <regex>
#include <unistd.h>

namespace OPENMMARM_SDK {

// ===== CRC32 实现（与 openmmarm_hw/arm_sdk_client.cpp 中使用的 crc32
// 一致）=====
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91B, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBE, 0xE7B82D09, 0x90BF1D9F, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F0B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0D6B, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7822, 0x3B6E20C8, 0x4C69105E,
    0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75,
    0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F0B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808,
    0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F,
    0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0D6B, 0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162,
    0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49,
    0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC,
    0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7822,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F6, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6B70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAE321249, 0xD9352200,
    0x60347344, 0x17304367, 0xA4C2783B, 0xD3C14F0D, 0x4066380E, 0x37612C98,
    0x0A00AE27, 0x7D079EB1, 0xE40ECF0B, 0x9309FF9D, 0x0D6C823E, 0x7A6BA2A8,
    0xE363B212, 0x94648284, 0x04DBB115, 0x73DC8193, 0xEAD54739, 0x9DD277AF,
    0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
};

uint32_t UdpPort::crc32(const void *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < length; ++i) {
    crc = (crc >> 8) ^ crc32_table[(crc ^ bytes[i]) & 0xFF];
  }
  return crc ^ 0xFFFFFFFF;
}

bool UdpPort::checkCRC32(uint8_t *data, size_t len) {
  uint32_t computed = crc32(data, len - sizeof(uint32_t));
  uint32_t received;
  std::memcpy(&received, &data[len - sizeof(uint32_t)], sizeof(uint32_t));
  return computed == received;
}

void UdpPort::addCRC32(uint8_t *data, size_t len) {
  uint32_t crc_value = crc32(data, len - sizeof(uint32_t));
  std::memcpy(data + len - sizeof(uint32_t), &crc_value, sizeof(uint32_t));
}

bool UdpPort::isValidIPv4(const std::string &ip) {
  const std::regex pattern(
      R"(^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)");
  return std::regex_match(ip, pattern);
}

// ===== Server 构造函数 =====
UdpPort::UdpPort(std::string name, unsigned int ownPort, size_t timeout_ms)
    : name_(std::move(name)), timer_(1.0), is_server_(true) {
  // 设置单次非阻塞接收超时
  timeout_saved_.tv_sec = static_cast<time_t>(timeout_ms / 1000);
  timeout_saved_.tv_usec =
      static_cast<suseconds_t>((timeout_ms * 1000) % 1000000);

  // 本地地址
  std::memset(&own_addr_, 0, sizeof(own_addr_));
  own_addr_.sin_family = AF_INET;
  own_addr_.sin_port = htons(static_cast<uint16_t>(ownPort));
  own_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

  std::memset(&target_addr_, 0, sizeof(target_addr_));
  std::memset(&from_addr_, 0, sizeof(from_addr_));

  // 创建 socket
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "[ERROR] UdpPort: " << name_ << " socket 创建失败\n";
    return;
  }

  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_on_,
             sizeof(reuse_on_));

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&own_addr_),
           sockaddr_size_) < 0) {
    std::cerr << "[ERROR] UdpPort: " << name_ << " bind 端口 " << ownPort
              << " 失败\n";
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

// ===== Client 构造函数 =====
UdpPort::UdpPort(std::string name, unsigned int ownPort, std::string toIP,
                 unsigned int toPort, size_t timeout_ms)
    : name_(std::move(name)), timer_(1.0), is_server_(false) {
  if (!isValidIPv4(toIP)) {
    std::cerr << "[ERROR] UdpPort: 无效的 IP 地址: " << toIP << "\n";
    return;
  }

  // 设置单次非阻塞接收超时
  timeout_saved_.tv_sec = static_cast<time_t>(timeout_ms / 1000);
  timeout_saved_.tv_usec =
      static_cast<suseconds_t>((timeout_ms * 1000) % 1000000);

  // 本地地址
  std::memset(&own_addr_, 0, sizeof(own_addr_));
  own_addr_.sin_family = AF_INET;
  own_addr_.sin_port = htons(static_cast<uint16_t>(ownPort));
  own_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

  // 目标地址
  std::memset(&target_addr_, 0, sizeof(target_addr_));
  target_addr_.sin_family = AF_INET;
  target_addr_.sin_port = htons(static_cast<uint16_t>(toPort));
  target_addr_.sin_addr.s_addr = inet_addr(toIP.c_str());

  std::memset(&from_addr_, 0, sizeof(from_addr_));

  // 创建 socket
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "[ERROR] UdpPort: " << name_ << " socket 创建失败\n";
    return;
  }

  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_on_,
             sizeof(reuse_on_));

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&own_addr_),
           sockaddr_size_) < 0) {
    std::cerr << "[ERROR] UdpPort: " << name_ << " bind 端口 " << ownPort
              << " 失败\n";
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

// ===== 发送 =====
IOPortStatus UdpPort::send(uint8_t *sendMsg, size_t sendLength) {
  if (socket_fd_ < 0)
    return IOPortStatus::ERROR;

  if (crc32_enabled_) {
    addCRC32(sendMsg, sendLength);
  }

  if (ntohs(target_addr_.sin_port) == 0) {
    return IOPortStatus::ERROR;
  }

  ssize_t sent = sendto(socket_fd_, sendMsg, sendLength, 0,
                        reinterpret_cast<struct sockaddr *>(&target_addr_),
                        sockaddr_size_);

  return (static_cast<size_t>(sent) == sendLength) ? IOPortStatus::OK
                                                   : IOPortStatus::ERROR;
}

// ===== 接收不定长 =====
size_t UdpPort::recvLen(uint8_t *recvMsg, size_t maxLength) {
  if (socket_fd_ < 0)
    return 0;
  ssize_t n = recvfrom(socket_fd_, recvMsg, maxLength, MSG_DONTWAIT,
                       reinterpret_cast<struct sockaddr *>(&from_addr_),
                       &sockaddr_size_);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

// ===== 接收定长 =====
IOPortStatus UdpPort::recv(uint8_t *recvMsg, size_t recvLength) {
  if (socket_fd_ < 0)
    return IOPortStatus::ERROR;

  size_t received_len = 0;

  // 非阻塞接收（使用 select）
  fd_set rset;
  FD_ZERO(&rset);
  FD_SET(socket_fd_, &rset);
  timeval timeout = timeout_saved_;

  int sel = select(socket_fd_ + 1, &rset, nullptr, nullptr, &timeout);

  if (sel < 0) {
    return IOPortStatus::ERROR;
  }

  if (sel == 0) {
    // 超时
    if (is_connected_) {
      if (timer_.waitTime() < 0) {
        is_connected_ = false;
        std::cout << "[ERROR] 与 UDP " << name_ << " 的连接已断开\n";
        if (is_server_) {
          target_addr_.sin_port = htons(0);
        }
      }
    }
    return IOPortStatus::TIMEOUT;
  }

  // 有数据可读 - 刷新连接定时器
  timer_.start();

  // 清空接收缓存，只取最新的数据
  timeval zero_timeout{0, 0};
  fd_set clear_set;
  FD_ZERO(&clear_set);
  FD_SET(socket_fd_, &clear_set);

  while (true) {
    if (select(socket_fd_ + 1, &rset, nullptr, nullptr, &zero_timeout) <= 0) {
      break;
    }
    received_len = static_cast<size_t>(recvfrom(
        socket_fd_, recv_buffer_.data(), recvLength, MSG_DONTWAIT,
        reinterpret_cast<struct sockaddr *>(&from_addr_), &sockaddr_size_));
  }

  // 重连检测
  if (!is_connected_) {
    is_connected_ = true;
    std::cout << "[REPORT] 已重新连接 UDP " << name_ << "\n";
  }

  // 校验长度
  if (received_len != recvLength) {
    std::cout << "[WARNING] UDP " << name_ << " 收到 " << received_len
              << " 字节，预期 " << recvLength << " 字节\n";
    return IOPortStatus::MSG_BAD;
  }

  // CRC 校验
  if (crc32_enabled_ && !checkCRC32(recv_buffer_.data(), received_len)) {
    return IOPortStatus::CRC_BAD;
  }

  // 复制数据
  std::memcpy(recvMsg, recv_buffer_.data(), received_len);

  // Server 模式下自动设定目标地址
  if (ntohs(target_addr_.sin_port) == 0) {
    target_addr_ = from_addr_;
  }

  return IOPortStatus::OK;
}

} // namespace OPENMMARM_SDK
