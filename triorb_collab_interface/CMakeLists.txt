cmake_minimum_required(VERSION 3.5)
project(triorb_collab_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(triorb_collaboration_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB msg_files RELATIVE "${CMAKE_SOURCE_DIR}" "msg/*.msg")
file(GLOB srv_files RELATIVE "${CMAKE_SOURCE_DIR}" "srv/*.srv")
file(GLOB action_files RELATIVE "${CMAKE_SOURCE_DIR}" "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs triorb_collaboration_interface
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
