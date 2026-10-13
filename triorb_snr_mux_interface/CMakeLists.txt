cmake_minimum_required(VERSION 3.8)
project(triorb_snr_mux_interface)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SnrMuxFrame.msg"
  "msg/SnrMuxStatus.msg"
  "msg/AuxOutput.msg"
  "msg/AuxSoundOnce.msg"
  "srv/SetAuxOutput.srv"
  "srv/PlayAuxSoundOnce.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
