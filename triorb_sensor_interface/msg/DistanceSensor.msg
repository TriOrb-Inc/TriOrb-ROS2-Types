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
float32 distance      		# Distance to obstacle [m]
uint8 confidence            # Signal reliability (0-100)
float32 hfov                # Horizontal detectable angle [deg]
float32 vfov                # Vertical detectable angle [deg]
float32 max_dist            # Maximum detectable distance [m]
float32 min_dist            # Minimum detectable distance [m]
float32[] mount_xyz         # Mounting location [deg]
float32[] mount_ypr         # Mounting orientation [deg]