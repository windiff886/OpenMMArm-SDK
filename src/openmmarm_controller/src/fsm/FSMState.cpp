#include "fsm/FSMState.h"

FSMState::FSMState(std::shared_ptr<CtrlComponents> ctrlComp,
                   ArmFSMState stateEnum, const std::string &stateName)
    : ctrlComp_(ctrlComp), stateEnum_(stateEnum), stateName_(stateName) {}
