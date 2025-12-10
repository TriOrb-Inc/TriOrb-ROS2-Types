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

#==PLCへ送信する基本データ==
std_msgs/Header header              # timestamp
uint8 index                         # Assembly index of the byte array
bool watchdog_request_from_jetson   # JetsonからPLCへのウォッチドッグ要求
bool watchdog_response_to_plc       # PLCへのウォッチドッグ応答
bool emergency_stop_to_plc          # PLCへの非常停止要求(B接点)
bool deactivate_request_to_plc      # PLCへの管理停止要求(A接点)
bool sls_off_request_to_plc         # PLCへのSLS監視停止要求(A接点)
bool error_reset_request_to_plc     # PLCへのエラーリセット要求(A接点)
uint8 reserved                      # 予約（未使用ビット）
