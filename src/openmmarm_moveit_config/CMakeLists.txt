cmake_minimum_required(VERSION 3.8)
project(openmmarm_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装配置和启动文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
