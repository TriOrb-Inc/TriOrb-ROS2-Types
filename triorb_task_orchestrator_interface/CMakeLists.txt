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

# Generate only interfaces whose V2 fields are defined by data_difinision.md.
# Legacy Route/Event/Result interfaces remain as source drafts but are not part
# of the generated V2 contract. Their compatibility/adapter rollout is unresolved
# by U-23 and is not guessed in this type-only change.
set(msg_files
  "msg/ActionExecutionKey.msg"
  "msg/ActionExecutionReport.msg"
  "msg/ActionExecutionReportDetail.msg"
  "msg/ActionSequence.msg"
  "msg/MapFileInfo.msg"
  "msg/RawActionSequence.msg"
  "msg/RobotAction.msg"
  "msg/TaskExecutionStatus.msg"
  "msg/TaskFileInfo.msg"
)
set(srv_files
  "srv/ActionExecutionCommand.srv"
  "srv/ReconcileExecution.srv"
  "srv/ResolveTask.srv"
)
set(action_files
  "action/ExecuteTaskFile.action"
)

# TODO(U-23): TaskResult.msg has no settled fields and is intentionally excluded
# from rosidl generation until its task-level status/reason contract is fixed.
# The V2 request/response fields for DeleteTaskFile, GetCurrentTask, GetEvents,
# ListTasks, SaveTask, GetTask, ListMapFiles, DeleteMapFile, and GetResults are
# also undefined. Existing matching files are legacy drafts; the four new source
# placeholders are likewise excluded rather than publishing guessed empty APIs.
# ExecuteTask.action is also excluded because its nested sequence cardinality and
# inline Compiler path are unresolved. SetTaskExecutionCommand.srv is excluded
# because its task/request correlation and structured response reason are P0.
# ActionExecution.action is excluded because whether its goal includes task_id is
# likewise an unresolved P0 wire-shape decision.
# TODO(U-23): The legacy nav/state projection, Action Ability schema, and
# absolute-versus-relative endpoint names also remain outside this type package
# until their source metadata and deployment contracts are fixed.
# TODO(U-24): The lower-module interfaces for action types 310 and 910 are not
# defined by these documents and are therefore not guessed in this package.

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
