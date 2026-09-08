cmake_minimum_required(VERSION 3.5)
project(triorb_asset_interface)

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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MapFileInfo.msg"
  "msg/TaskFileInfo.msg"
)
set(srv_files
  "srv/DeleteMapFile.srv"
  "srv/DeleteTaskFile.srv"
  "srv/DownloadTaskFile.srv"
  "srv/GetTask.srv"
  "srv/GetTaskFileList.srv"
  "srv/ListMapFiles.srv"
  "srv/SaveTask.srv"
  "srv/UploadTaskFile.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
