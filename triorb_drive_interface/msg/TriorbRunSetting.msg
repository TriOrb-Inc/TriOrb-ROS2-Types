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

#==自律移動の位置決め設定==
float32 tx                  # Target error in X-axis direction [±m]
float32 ty                  # Target error in Y-axis direction [±m].
float32 tr                  # Target error in rotation [±deg].
uint8 force                 # Target force level
uint8 gain_no               # Number of gain type (not set:0, basic:1)
uint32 timeout_ms           # Timeout (in ms) for the operation to complete (set:0, disable)
uint8[] disable_camera_idx  # Camera Index to be excluded from robot pose estimation