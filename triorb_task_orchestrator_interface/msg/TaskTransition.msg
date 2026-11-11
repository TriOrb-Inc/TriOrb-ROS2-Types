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

# 一度だけ通知するtask遷移。型の契約は../INTERFACE.mdを参照。
uint8 KIND_NAVIGATION_STARTED=1
uint8 KIND_TASK_SUCCEEDED=2
uint8 KIND_TASK_FAILED=3
uint8 KIND_TASK_CANCELLED=4

builtin_interfaces/Time stamp  # 同じ評価で作成したTaskObservation.stamp
string<=64 source_session     # TaskObservation.source_session
uint64 observation_sequence   # 同じ評価で作成したTaskObservation.observation_sequence
uint64 task_generation        # 遷移対象のTaskObservation.task_generation
uint8 kind                    # このmsgのKIND_*（0は無効）
bool has_action               # action_type_id/action_phaseが有効な場合にtrue
uint16 action_type_id          # 遷移対象のRobotAction.action_type_id
uint8 action_phase             # ActionExecutionReportDetail.PHASE_*
uint32 reason_code             # ActionExecutionReportDetail.REASON_*
