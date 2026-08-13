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

#==PLCから受信した非常停止詳細データ==
std_msgs/Header header              # timestamp
uint8 index                         # Assembly index of the byte array
uint8 power_cutted                  # 動力遮断されている
uint8 switch_pushed                 # 非常停止ボタンが押されている
uint8 sls_detected                  # SLSが障害物検知している
uint8 remote_switch_disabled        # リモコン非常停止状態
uint8 watchdog_error                # ウォッチドッグ監視エラー
uint8 estop_from_jetson             # Jetsonからの非常停止要求あり
uint8 reserved                      # 予約（未使用ビット）
