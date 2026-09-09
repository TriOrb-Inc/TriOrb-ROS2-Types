cmake_minimum_required(VERSION 3.5)
project(triorb_task_orchestrator_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(triorb_drive_interface REQUIRED)

set(msg_files
  "msg/ActionExecutionKey.msg"
  "msg/ActionExecutionReport.msg"
  "msg/ActionExecutionReportDetail.msg"
  "msg/ActionSequence.msg"
  "msg/RawActionSequence.msg"
  "msg/RobotAction.msg"
  "msg/TaskEvent.msg"
  "msg/TaskExecutionState.msg"
  "msg/TaskResult.msg"
)
set(srv_files
  "srv/GetCurrentTask.srv"
  "srv/GetRawActionSequence.srv"
  "srv/GetTaskEvents.srv"
  "srv/GetTaskResults.srv"
  "srv/SetActionExecutionCommand.srv"
  "srv/SetTaskExecutionCommand.srv"
)
set(action_files
  "action/ExecuteAction.action"
  "action/ExecuteTask.action"
  "action/ExecuteTaskFile.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces triorb_drive_interface
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
