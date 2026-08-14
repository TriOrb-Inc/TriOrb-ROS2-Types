cmake_minimum_required(VERSION 3.8)
project(triorb_task_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(triorb_drive_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskStep.msg"
  "action/ExecuteSequence.action"
  "srv/SetExecutorCommand.srv"
  DEPENDENCIES triorb_drive_interface
)

ament_export_dependencies(rosidl_default_runtime triorb_drive_interface)
ament_package()
