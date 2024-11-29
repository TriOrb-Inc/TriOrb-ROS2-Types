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

#==ホストコンピューターのモニター==
std_msgs/Header header      # Timestamp
float32 memory_percent      # Memory usage
float32 cpu_percent         # CPU usage
float32 host_temperature    # Temperature of the host computer
string wlan_ssid            # SSID of the access point
uint8 wlan_signal           # Signal strength of the access point 
uint32 wlan_freq            # Communication speed of the access point
float32 ping                # Ping speed to the default gateway
uint8[] gateway             # Address of the default gateway