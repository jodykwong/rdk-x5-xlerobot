cmake_minimum_required(VERSION 3.8)
project(xlerobot_online_dialogue)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成ROS2消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OnlineDialogueInput.msg"
  "msg/OnlineDialogueResponse.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python可执行文件
install(PROGRAMS
  xlerobot_online_dialogue/online_dialogue_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(audio_msg)

ament_package()