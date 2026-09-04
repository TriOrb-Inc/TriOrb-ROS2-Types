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

# action_instance_idとactionsは同じ配列長でなければならず、同じ配列indexで対応する。
# byte数、行数、field長はIDLで制限しない。DDS resource limit確認とchunkingはこの契約の対象外。
uint32[] action_instance_id  # actionsと同じ配列indexで対応
RobotAction[] actions
