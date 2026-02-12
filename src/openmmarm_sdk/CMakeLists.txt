cmake_minimum_required(VERSION 3.8)
project(openmmarm_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ===== 共享库 =====
add_library(openmmarm_sdk SHARED
  src/udp_port.cpp
)

target_include_directories(openmmarm_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ===== 示例程序 =====
set(EXAMPLE_FILES
  examples/example_joint_ctrl.cpp
  examples/example_lowcmd.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} openmmarm_sdk)
endforeach()

# ===== 安装 =====
install(TARGETS openmmarm_sdk
  EXPORT openmmarm_sdkTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装示例程序
foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  install(TARGETS ${EXAMPLE_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# 导出供下游使用
ament_export_targets(openmmarm_sdkTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(openmmarm_sdk)
ament_export_dependencies(Eigen3)

ament_package()
