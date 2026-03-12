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


std_msgs/Header header          # header.stamp: 計測/受信時刻
                                # header.frame_id: 任意（例 "battery"）

uint32 remaining_mah            # 現在の残量 [mAh]
float32 remaining_percent       # 残容量 [%] (0.0〜100.0)
float32 total_voltage_v         # バッテリー全体電圧 [V]
bool is_charging                # 充電Flag
BatteryModuleStatus[] modules
