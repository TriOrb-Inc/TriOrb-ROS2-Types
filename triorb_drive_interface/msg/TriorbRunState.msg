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

#==自律移動pkgの状態通知トピック==
TriorbPos3 goal_pos     # Latest goal position
uint8 state             # Navigate state

#---state values---
# 0: stand by ( No topic subscribed or subscribe /drive/stop )
# 1: navigating now
# 2: pause ( Subscribe /drive/pause )
# 3: success ( Success navigation. )
# 4: failed ( Failed navigation. Mostly lost. )
# 5: leave goal ( Success navigation. But, leave from goal_pos. )