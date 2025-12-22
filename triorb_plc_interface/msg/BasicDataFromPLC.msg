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

#==PLCから受信した基本データ==
std_msgs/Header header              # timestamp
uint8 index                         # Assembly index of the byte array
bool watchdog_request_from_plc      # PLCからのウォッチドッグ要求
bool watchdog_response_from_plc     # PLCからのウォッチドッグ応答
bool emergency_stop_from_plc        # PLCが非常停止中(B接点)
bool power_cut_from_plc             # PLCが動力遮断中(B接点)
bool unknown_error_from_plc         # PLCからの不明なエラー(A接点)
bool permit_auto_move_from_plc      # PLCからの自動移動許可信号(B接点)
bool permit_manual_move_from_plc    # PLCからの自動移動許可信号(B接点)
bool sls_off_from_plc               # PLCからのSLS監視停止状態信号(A接点)