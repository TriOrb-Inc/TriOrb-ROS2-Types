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

# taskの実行状態をGUIやFMSへ通知するための情報。Orchestrator process内でのdispatch IDを含む。

string current_state
uint32 loop               # taskのloop回数。0はloopなし、1以上はloopあり。
uint32 current_loop_index # 現在実行中のloop数
uint32 action_instance_id # task中の実行順を表わす

TaskFileInfo task_file_info  # 実行中のtask csvファイルの情報。実行中にtask csvファイルが変更される可能性があるため、task csvファイルの情報を含める。