cmake_minimum_required(VERSION 3.8)
project(openmmarm_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(openmmarm_sdk REQUIRED)

# 包含目录
include_directories(
  include
)

# 硬件接口库
add_library(${PROJECT_NAME} SHARED
  src/openmmarm_hw.cpp
  src/arm_sdk_client.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rclcpp_action
  control_msgs
  openmmarm_sdk
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface openmmarm_hw.xml)

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rclcpp_action
  control_msgs
  openmmarm_sdk
)

ament_package()
