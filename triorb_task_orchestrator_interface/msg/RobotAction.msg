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
string value     # actionに対する指令/action分類などactionにより仕様が異なる

triorb_drive_interface/TriorbSetPos3 target  # 目標位置・姿勢と走行設定
