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

#==PLCへ送信するアプリケーションデータ==
std_msgs/Header header              # timestamp
uint8 index                         # Assembly index of the byte array
bool is_moving                      # 移動中
bool lifter_moving                  # リフター動作中
bool has_error                      # エラー発生中
bool battery_low                    # バッテリー残量低下
bool is_collab                      # 協調中
bool lifter_up                      # リフターUP状態
bool lifter_down                    # リフターDOWN状態
uint8 reserved                      # 予約（未使用ビット）
