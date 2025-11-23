cmake_minimum_required(VERSION 3.8)
project(audio_msg)

if(CMAKE_COMPILER_IS_GNUCCXE OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/AudioConfig.msg"
  "msg/AudioStatus.msg"
  "msg/ASRResult.msg"
  "msg/ASRStatus.msg"
  "msg/LLMResponse.msg"
  "msg/LLMStatus.msg"
  "msg/TTSStatus.msg"
  "srv/SetAudioDevice.srv"
  "srv/AudioConfigure.srv"
  "srv/ASRConfigure.srv"
  "action/ContinuousRecognition.action"
  DEPENDENCIES
    std_msgs
    action_msgs
)

# Register interfaces
ament_export_dependencies(rosidl_default_runtime)

# Install
ament_package()