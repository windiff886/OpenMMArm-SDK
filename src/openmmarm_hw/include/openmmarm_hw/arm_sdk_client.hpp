#ifndef OPENMMARM_ARM_SDK_CLIENT_HPP
#define OPENMMARM_ARM_SDK_CLIENT_HPP

#include "openmmarm_sdk/openmmarm_arm_common.h"

#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace openmmarm_hw {

/**
 * @brief 机械臂 SDK 客户端
 *
 * 封装 UDP 客户端逻辑，与 openmmarm_controller 的 CmdSdk（UdpSdk）通信。
 * 设计参考 z1_sdk 中的 UnitreeArm 类。
 *
 * 使用方式:
 *   ArmSdkClient client("127.0.0.1", 8072, 8871);
 *   client.init();
 *   client.armCmd.mode = (uint8_t)ArmMode::JOINT_CTRL;
 *   client.armCmd.q_d = {0.0, 0.1, ...};
 *   client.sendRecv();
 *   // 读取 client.armState.q 获取当前位置
 */
class ArmSdkClient {
public:
  /**
   * @brief 构造函数
   * @param controller_ip  openmmarm_controller 的 IP 地址
   * @param local_port     本地绑定端口
   * @param controller_port  controller 的 CmdSdk 监听端口（默认 8871）
   */
  ArmSdkClient(const std::string &controller_ip, int local_port,
               int controller_port);

  ~ArmSdkClient();

  /**
   * @brief 初始化 socket 并连接
   * @return 成功返回 true
   */
  bool init();

  /**
   * @brief 发送 ArmCmd 并接收 ArmState
   *
   * 先发送当前的 armCmd，然后尝试接收 controller 回传的 armState。
   * 这与 z1 的 UnitreeArm::sendRecv() 模式一致。
   */
  void sendRecv();

  /**
   * @brief 检查是否已连接（曾经收到过 ArmState）
   */
  bool isConnected() const { return connected_.load(); }

  // 上行指令（hw → controller）
  OPENMMARM_SDK::ArmCmd armCmd{};

  // 下行状态（controller → hw）
  OPENMMARM_SDK::ArmState armState{};

private:
  std::string controller_ip_;
  int local_port_;
  int controller_port_;
  int sockfd_{-1};

  struct sockaddr_in controller_addr_ {};

  std::atomic<bool> connected_{false};

  // CRC32 计算
  static uint32_t crc32(const void *data, size_t len);
};

} // namespace openmmarm_hw

#endif // OPENMMARM_ARM_SDK_CLIENT_HPP
