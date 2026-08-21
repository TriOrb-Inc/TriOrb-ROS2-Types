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

#==自律移動結果==
bool success                # Moving result (true: Compleat, false: Feild)
uint8 info                  # Moving result info ( substitution NAVIGATE_RESULT )
TriorbPos3 position         # Last robot position

#---Moving result info---
# 0: TIMEOUT_FAILED（タイムアウト）
# 1: HALF_TIMEOUT
# 2: TRANSFORM_FAILED
# 3: NO_CHANGE_TIMESTAMP
# 4: FORCE_STOP（/drive/stopなどで強制停止）
# 5: NAVIGATION_FAILED
# 6: NAVIGATION_SUCCESS
# 7: PROGRESS
# 8: FORCE_SUCCESS（force success指示）
# 9: LOST_FAILED（ロスト判定）
# 255: REJECT（移動中に来た指示や不正引数を拒否）
