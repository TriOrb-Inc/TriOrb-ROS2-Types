cmake_minimum_required(VERSION 3.8)
project(triorb_aux_interface)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommandConstants.msg"
  "msg/AuxOutput.msg"
  "msg/AuxSoundOnce.msg"
  "srv/SetAuxOutput.srv"
  "srv/PlayAuxSoundOnce.srv"
  DEPENDENCIES builtin_interfaces unique_identifier_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_serialization test/test_serialization.py TIMEOUT 60)
endif()

ament_package()
