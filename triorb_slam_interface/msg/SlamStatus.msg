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

#==SLAMの実行状況==
string map_name  # map name
uint8 state      # mapping now, fix map... etc
uint8 error      # not working, map load failed... etc

#---slam operation status (bit flag)---
# 地図更新モード(0)ではなく固定地図モード(1)で動作している。
uint8 STATE_MAP_FIXED=1
# 自己位置推定が成立している。
uint8 STATE_LOCALIZED=2
# 地図保存処理中。
uint8 STATE_SAVING_MAP=4
# 地図読込処理中。
uint8 STATE_LOADING_MAP=8
# ユーザー操作などでSLAM本体を意図的に休止している。
uint8 STATE_PAUSED=16
# 地図ファイルの読込が完了しており、自己位置推定に使える地図名が確定している。
uint8 STATE_MAP_LOADED=32
# 地図読込完了後、自己位置推定が未成立または途切れている。
uint8 STATE_LOST=64

#---error state (bit flag)---
# SLAM本体が意図せず起動していない。
uint8 ERROR_NOT_WORKING=1
# 現在の起動または地図読込後に、まだ一度も自己位置推定が成立していない。
uint8 ERROR_NEVER_LOCALIZED=2
# 地図保存に失敗した。
uint8 ERROR_MAP_SAVE_FAILED=4
# 地図読込に失敗した。
uint8 ERROR_MAP_LOAD_FAILED=8
