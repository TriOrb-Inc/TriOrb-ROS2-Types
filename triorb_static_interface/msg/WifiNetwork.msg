#**
#* Copyright 2026 TriOrb Inc.
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

#==Wi-Fiスキャン結果 1件==
# Mirrors triorb.system.v1.WifiNetwork (triorb-system-core network.proto)
string ssid            # SSID
string bssid           # BSSID (AP MAC address)
uint8 signal_strength  # 0..100
string security        # e.g. "WPA2", "" = open
uint32 frequency_mhz   # channel frequency [MHz]
bool in_use            # currently connected AP
