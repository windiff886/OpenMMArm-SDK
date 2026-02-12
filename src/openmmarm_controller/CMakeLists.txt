cmake_minimum_required(VERSION 3.8)
project(openmmarm_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(openmmarm_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(openmmarm_sdk REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 可执行文件
add_executable(openmmarm_ctrl
  src/main.cpp
  src/cmd/CmdSdk.cpp
  src/cmd/UdpSdk.cpp
  src/io/IOROS.cpp
  src/io/IOUDP.cpp
  src/fsm/FiniteStateMachine.cpp
  src/fsm/FSMState.cpp
  src/fsm/State_Passive.cpp
  src/fsm/State_JointCtrl.cpp
)

# 链接依赖
ament_target_dependencies(openmmarm_ctrl
  rclcpp
  ament_index_cpp
  openmmarm_interfaces
  openmmarm_sdk
)

# 不需要额外链接库
# target_link_libraries(openmmarm_ctrl)

# 安装
install(TARGETS openmmarm_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
