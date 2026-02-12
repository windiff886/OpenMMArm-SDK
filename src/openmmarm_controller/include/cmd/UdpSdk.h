#pragma once

#include "cmd/CmdSdk.h"
#include "ctrl/CtrlComponents.h"
#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

/**
 * @brief 基于 UDP 的上层指令接口实现
 *
 * 在指定端口上创建 UDP Server，等待外部 SDK 客户端连接。
 * 支持非阻塞接收，不会阻塞控制循环。
 *
 * 通信协议:
 * - 接收: ArmCmd（packed 结构体，包含 CRC32 校验）
 * - 发送: ArmState（packed 结构体，包含 CRC32 校验）
 */
class UdpSdk : public CmdSdk {
public:
  /**
   * @brief 构造函数
   * @param port 监听端口（默认 8871）
   * @param lowState 底层状态指针（用于填充 ArmState）
   */
  UdpSdk(int port, std::shared_ptr<LowLevelState> lowState);
  ~UdpSdk() override;

  bool init() override;
  bool isConnected() override;
  void sendRecv() override;

private:
  // 从 lowState 填充 armState
  void updateArmState();

  // 从 armCmd 更新 fsmArmCmd（关节目标）
  void applyArmCmd();

  // CRC32 计算
  uint32_t calculateCRC32(const uint8_t *data, size_t len);

  // Socket
  int socket_fd_ = -1;
  int port_;

  // 客户端地址（记录最后一个发来数据的客户端）
  struct sockaddr_in client_addr_;
  socklen_t client_addr_len_ = sizeof(struct sockaddr_in);
  std::atomic<bool> has_client_{false};

  // 底层状态引用
  std::shared_ptr<LowLevelState> lowState_;

  // 接收超时
  static constexpr int RECV_TIMEOUT_MS = 5;
};
