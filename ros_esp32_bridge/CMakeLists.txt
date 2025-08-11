cmake_minimum_required(VERSION 3.8)
project(esp32_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# 创建可执行文件
add_executable(esp32_robot_bridge src/esp32_robot_bridge.cpp)

# 链接库
ament_target_dependencies(esp32_robot_bridge
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(esp32_robot_bridge
  ${Boost_LIBRARIES}
)

# 安装可执行文件
install(TARGETS
  esp32_robot_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
) 