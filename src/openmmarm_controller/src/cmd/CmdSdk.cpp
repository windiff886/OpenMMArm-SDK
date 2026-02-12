#include "cmd/CmdSdk.h"
#include "fsm/FSMState.h"

ArmFSMState CmdSdk::getTargetFSMState() const {
  // 将 ArmCmd 中的 mode(uint8_t) 映射到 ArmFSMState 枚举
  switch (static_cast<OPENMMARM_SDK::ArmMode>(armCmd.mode)) {
  case OPENMMARM_SDK::ArmMode::PASSIVE:
    return ArmFSMState::PASSIVE;
  case OPENMMARM_SDK::ArmMode::JOINT_CTRL:
    return ArmFSMState::JOINT_CTRL;
  case OPENMMARM_SDK::ArmMode::CARTESIAN:
    return ArmFSMState::CARTESIAN;
  case OPENMMARM_SDK::ArmMode::TEACH:
    return ArmFSMState::TEACH;
  case OPENMMARM_SDK::ArmMode::TEACH_REPEAT:
    return ArmFSMState::TEACH_REPEAT;
  case OPENMMARM_SDK::ArmMode::CALIBRATION:
    return ArmFSMState::CALIBRATION;
  case OPENMMARM_SDK::ArmMode::LOW_CMD:
    return ArmFSMState::LOW_CMD;
  default:
    return ArmFSMState::INVALID;
  }
}
