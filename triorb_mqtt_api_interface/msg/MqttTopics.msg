#**
#* Copyright 2026 TriOrb Inc.
#*
#* Licensed under the Apache License, Version 2.0 (the "License");
#* you may not use this file except in compliance with the License.
#* You may obtain a copy of the License at
#*
#*     http://www.apache.org/licenses/LICENSE-2.0
#*
#* Unless required by applicable law or agreed to in writing, software
#* distributed under the License is distributed on an "AS IS" BASIS,
#* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#* See the License for the specific language governing permissions and
#* limitations under the License.
#**

#==両方向: MQTT トピック名の定数==
# 実際のトピックは "<prefix>/" + 定数。<prefix> はロボットの識別子(/ を含まない 1 階層)
# ロボット → FMS
string STATE="mqtt_ros_bridge/state"                        # RobotState
string POSE="mqtt_ros_bridge/pose"                          # RobotPose
string TASK_STATE="mqtt_ros_bridge/task/state"              # TaskState
string STATE_RESPONSE="mqtt_ros_bridge/state/response"      # DiagnosticsSnapshot
string HEARTBEAT="mqtt_ros_bridge/heartbeat"                # Heartbeat(retain)
string COMMAND_RESULT="mqtt_ros_bridge/cmd/result"          # CommandResult
string PONG="mqtt_ros_bridge/pong"                          # 素の文字列(PING の折り返し)
# FMS → ロボット
string CMD_TASK_EXECUTE="mqtt_ros_bridge/cmd/task/execute"  # TaskExecuteCommand
string CMD_TASK_CONTROL="mqtt_ros_bridge/cmd/task/control"  # TaskControlCommand
string CMD_STOP="mqtt_ros_bridge/cmd/stop"                  # StopCommand
string STATE_REQUEST="mqtt_ros_bridge/state/request"        # DiagnosticsRequest
string PING="mqtt_ros_bridge/ping"                          # 素の文字列 "<送り主>:<エポックミリ秒>"
