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

uint16[] marker_id          # 位置合わせ原点とするマーカーのID
float32[] marker_size       # マーカーのサイズ
TriorbPos3[] position       # 原点に対する相対位置決め位置姿勢
TriorbSpeed speed           # 移動速度
TriorbRunSetting setting    # 走行設定

