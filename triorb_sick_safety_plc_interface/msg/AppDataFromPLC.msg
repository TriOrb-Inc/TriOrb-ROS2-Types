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

#==SICK Safety PLCから受信したアプリケーションデータ==
std_msgs/Header header              # timestamp
uint8 index                         # Assembly index of the byte array
bool error_reset_from_plc           # PLCからのエラーリセット要求
bool ready_to_drive                 # PLCからの運転準備完了信号
bool selected_auto_drive_mode       # PLCからの自動運転モード選択信号
bool selected_manual_drive_mode     # PLCからの手動運転モード選択信号
