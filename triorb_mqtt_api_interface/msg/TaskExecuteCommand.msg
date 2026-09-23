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

#==FMS → ロボット: タスク実行==
string request_id             # FMS が一意に振る
string file_name              # 保存済みタスクファイル名
uint32 loop                   # 周回数(1 以上)
float32 ratio_speed           # 速度倍率(0 より大)
float32 bias_accuracy_xy      # [m]
float32 bias_accuracy_deg     # [deg]
