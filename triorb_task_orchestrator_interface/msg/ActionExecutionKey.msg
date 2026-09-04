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

#==タスク実行の一意性を示すキー情報==
uint32 action_instance_id  # task内の論理位置
uint32 execution_id        # Orchestrator process内のdispatch ID。0は禁止。UINT32_MAX到達時はwrap-aroundする。
