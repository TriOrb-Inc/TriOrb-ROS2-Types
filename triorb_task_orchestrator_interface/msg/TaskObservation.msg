#**
#* Copyright 2023 TriOrb Inc.
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

# Task Orchestratorの正規化した状態snapshot。型の契約は../INTERFACE.mdを参照。
uint8 PHASE_IDLE=0
uint8 PHASE_COMPILING=1
uint8 PHASE_RUNNING=2
uint8 PHASE_PAUSING=3
uint8 PHASE_PAUSE_RESTART_STOPPING=4
uint8 PHASE_PAUSED=5
uint8 PHASE_FAILED_HOLD=6
uint8 PHASE_CANCELLING=7

builtin_interfaces/Time stamp  # snapshot生成時刻（ROS時刻）
string<=64 source_session     # 発行元プロセスの起動ごとに生成するUUID
uint64 observation_sequence   # heartbeatを含む各snapshotで増加、起動後の初回は1
uint64 task_generation        # task受付ごとに増加、初回受付前は0、IDLEでも最後の値を維持
uint8 task_phase               # このmsgのPHASE_*
bool has_action               # action_type_id/action_phaseが有効な場合にtrue
uint16 action_type_id          # RobotAction.action_type_idと同じ値
uint8 action_phase             # ActionExecutionReportDetail.PHASE_*
uint32 reason_code             # ActionExecutionReportDetail.REASON_*
