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

#==加減速時間・速度の設定==

# Time used to ramp up to the requested speed [ms]. New messages default to 1000 ms.
uint32 acc 1000
# Time used to ramp down from the requested speed [ms]. New messages default to 1000 ms.
uint32 dec 1000
# Maximum translational speed [m/s]. New messages default to 0.6 m/s.
float32 xy 0.6
# Maximum angular speed [rad/s]. New messages default to 0.6 rad/s.
float32 w 0.6
