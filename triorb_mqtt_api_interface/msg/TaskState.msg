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

#==ロボット → FMS: タスク実行の状態(変化時 + 実行中は 1 Hz)==
# TaskExecutionState の写しに最終結果を足したもの
builtin_interfaces/Time stamp
string current_state          # TaskExecutionState.current_state と同じ(取り得る値は task_orchestrator 側で未確定)
uint32 loop                   # 周回数
uint32 current_loop_index     # 現在の周回
uint32 action_instance_id     # 実行中の action の順番
string file_name

string FINAL_FINISHED="Finished"
string FINAL_ERROR="Error"
string FINAL_STOPPED="Stopped"
string FINAL_CANCELED="Canceled"
string final_status           # 最終結果(FINAL_*)。実行中は空
string message                # 失敗・停止の理由
