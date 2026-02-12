#include "openmmarm_hw/arm_sdk_client.hpp"

#include <cstring>
#include <fcntl.h>
#include <iostream>

namespace openmmarm_hw {

ArmSdkClient::ArmSdkClient(const std::string &controller_ip, int local_port,
                           int controller_port)
    : controller_ip_(controller_ip), local_port_(local_port),
      controller_port_(controller_port) {}

ArmSdkClient::~ArmSdkClient() {
  if (sockfd_ >= 0) {
    close(sockfd_);
    sockfd_ = -1;
  }
}

bool ArmSdkClient::init() {
  // 创建 UDP socket
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    std::cerr << "[ArmSdkClient] socket 创建失败" << std::endl;
    return false;
  }

  // 绑定本地端口
  struct sockaddr_in local_addr {};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(static_cast<uint16_t>(local_port_));

  if (bind(sockfd_, reinterpret_cast<struct sockaddr *>(&local_addr),
           sizeof(local_addr)) < 0) {
    std::cerr << "[ArmSdkClient] bind 端口 " << local_port_ << " 失败"
              << std::endl;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // 设置非阻塞模式
  int flags = fcntl(sockfd_, F_GETFL, 0);
  fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

  // 设置 controller 地址
  controller_addr_.sin_family = AF_INET;
  controller_addr_.sin_port = htons(static_cast<uint16_t>(controller_port_));
  inet_pton(AF_INET, controller_ip_.c_str(), &controller_addr_.sin_addr);

  // 设置接收超时（100ms）
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  std::cout << "[ArmSdkClient] 已初始化: " << controller_ip_ << ":"
            << controller_port_ << " (本地端口: " << local_port_ << ")"
            << std::endl;

  return true;
}

void ArmSdkClient::sendRecv() {
  if (sockfd_ < 0) {
    return;
  }

  // 1. 计算并填充 CRC
  armCmd.crc = crc32(&armCmd, sizeof(armCmd) - sizeof(uint32_t));

  // 2. 发送 ArmCmd 到 controller
  sendto(sockfd_, &armCmd, sizeof(armCmd), 0,
         reinterpret_cast<struct sockaddr *>(&controller_addr_),
         sizeof(controller_addr_));

  // 3. 尝试接收 ArmState
  OPENMMARM_SDK::ArmState recvBuf{};
  struct sockaddr_in from_addr {};
  socklen_t from_len = sizeof(from_addr);

  ssize_t n =
      recvfrom(sockfd_, &recvBuf, sizeof(recvBuf), 0,
               reinterpret_cast<struct sockaddr *>(&from_addr), &from_len);

  if (n == static_cast<ssize_t>(sizeof(OPENMMARM_SDK::ArmState))) {
    // 验证 CRC
    uint32_t expected_crc = crc32(&recvBuf, sizeof(recvBuf) - sizeof(uint32_t));
    if (recvBuf.crc == expected_crc) {
      armState = recvBuf;
      connected_ = true;
    }
  }
}

uint32_t ArmSdkClient::crc32(const void *data, size_t len) {
  const uint8_t *buf = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

} // namespace openmmarm_hw
