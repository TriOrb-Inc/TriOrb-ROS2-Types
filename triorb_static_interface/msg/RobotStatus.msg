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

#==ロボットの状態==
std_msgs/Header header  # timestamp
float32 voltage         # main power supply voltage
uint16 btns             # Remote control operation status (bit flag)
uint16 state            # Robot operation state (bit flag)
uint16 error            # Error status of the robot (bit flag)

uint32 ken_flags1
uint32 ken_errors
uint32 ken_warns

#---Remote control operation status (bit flag)---
# 0x8000: Remote control Y button
# 0x4000: Remote control B button
# 0x2000: Remote control A button
# 0x1000: Remote control X button

#---Robot operation state (bit flag)---
# 0x8000: Motor is being excited
# 0x4000: Accepting move instruction
# 0x2000: Moving
# 0x1000: Self-position recognition in progress
# 0x0800: Generating map
# 0x0400: During anti-collision control
# 0x0200: Position control move completed

#---Error status of the robot (bit flag)---
# 0x8000: Motor connection error
# 0x4000: IMU and distance sensor connection error
# 0x2000: Camera connection error
# 0x1000: Main power supply voltage abnormal