cmake_minimum_required(VERSION 3.8)
project(triorb_snr_mux_interface)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SnrMuxFrame.msg"
  "msg/SnrMuxStatus.msg"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_serialization test/test_serialization.py TIMEOUT 60)
endif()

ament_package()
