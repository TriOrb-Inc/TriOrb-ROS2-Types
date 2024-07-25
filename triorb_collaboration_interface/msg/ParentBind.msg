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

# ==[協調搬送] 仮想（荷物など）原点に対するロボットの相対姿勢==
std_msgs/Header header      # Header
string parent               # Parent name
string you                  # Your name
float32 x                   # Relative position of the parent from you [m]
float32 y                   # Relative position of the parent from you [m]
float32 deg                 # Relative position of the parent from you [deg]
