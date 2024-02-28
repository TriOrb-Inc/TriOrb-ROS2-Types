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

std_msgs/Header header      # Timestamp
uint16 last_error_value     # Last motor alert flag
uint8 last_error_motor      # Motor ID of the last alert
float32 voltage             # Mains voltage observed by the motor driver
uint16 state                # Operating state of each motor (bit flag)
float32 power               # Power consumption of each motor (W)

#---Operating state of each motor (bit flag)---
# 0x8000: Remote control Y button
# 0x4000: Remote control B button
# 0x2000: Remote control A button
# 0x1000: Remote control X button
# 0x0800: Rotating
# 0x0400: Position control complete
# 0x0200: Excitation in progress
# 0x0100: Motor status acquired successfully