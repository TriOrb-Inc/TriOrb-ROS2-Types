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
# 0b00000001: mapping mode(0), fix map(1)
# 0b00000010: lost(0), localize(1)
# 0b00000100: processing save map 
# 0b00001000: processing load map
 
#---error state (bit flag)---
# 0b00000001: slam is not working
# 0b00000010: Never detected a known landmark
# 0b00000100: save map failed
# 0b00001000: load map failed (cannot find map)