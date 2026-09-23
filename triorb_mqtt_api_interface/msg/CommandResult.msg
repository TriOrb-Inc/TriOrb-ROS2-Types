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

#==ロボット → FMS: コマンドへの応答==
# 受理時と完了時に出る
builtin_interfaces/Time stamp
string request_id             # コマンドの request_id
bool accepted                 # 受理した
bool success                  # ROS 2 側の呼び出しが成功した
bool confirmed                # stop のみ。停止を観測した
string message                # 拒否・失敗の理由
