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

# ActionSequence.msg/RawActionSequence.msgのactions配列の要素型
# task csvファイルやjsonデータからの字句変換済みデータ
uint16 action_type_id  # actionの実行種別
string type      # GUI用i18nextライブラリの翻訳キー.例外としてcollab00の際は26552200として、下2桁がgroup noを表わす
string name      # GUI用の表示名
string value     # GUI用の表示区分名

float64 x        # 目標位置 [m]
float64 y        # 目標位置 [m]
float64 yaw      # 目標姿勢 [deg]

float64 vxy      # 並進速度 [m/s]
float64 vw       # 回転速度 [rad/s]

float64 acc_xy   # 位置の許容誤差 [m]
float64 acc_w    # 姿勢の許容誤差 [deg]

uint8 gain       # TriorbRunSetting/gain_no (not set:0, basic:1)
uint32 accel_ms  # 加減速時間 [ms]。**0 は未指定**として扱い、既定の 1000 が使われる
string camera    # "1|0|1|1" 形式。0 の位置のカメラを測位から外す。空なら全て使う
