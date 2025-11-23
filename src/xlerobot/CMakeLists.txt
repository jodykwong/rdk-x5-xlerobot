cmake_minimum_required(VERSION 3.8)
project(xlerobot)

if(CMAKE_COMPILER_IS_GNUCXE OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(audio_msg REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts with executable names
install(PROGRAMS
  nodes/asr_bridge_node.py
  nodes/llm_service_node.py
  nodes/tts_service_node.py
  nodes/voice_assistant_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install to libexec for ROS2 discovery
install(PROGRAMS
  nodes/asr_bridge_node.py
  nodes/llm_service_node.py
  nodes/tts_service_node.py
  nodes/voice_assistant_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  ../../config
  DESTINATION share/${PROJECT_NAME}/config
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()