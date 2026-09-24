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

#==ロボット → FMS: ロボットの状態(1 Hz)==
# FMS が統一状態コードを導出するための値。統一状態コードそのものは載せない
builtin_interfaces/Time stamp

# ECU の状態(robot/ecu_status)
bool facts_fresh              # ecu_status が鮮度の上限(Bridge のパラメータ。既定 5 秒)以内に取れている。false なら下の bool は無効
bool emergency_stop           # 非常停止中
bool moving                   # 走行中
bool motor_energized          # 励磁中
bool brake_free               # 電磁ブレーキ解放

# タスク
bool navigating               # タスク実行中
string active_task_file       # 実行中のタスクファイル名。無ければ空

# 電源
float32 voltage               # ECU の電源電圧 [V](robot/ecu_status)。未受信なら 0
bool charging                 # 充電中(battery/status)
