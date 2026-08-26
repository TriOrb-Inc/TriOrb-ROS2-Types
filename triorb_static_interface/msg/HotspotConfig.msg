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

#==Hotspot configuration==
# Mirrors triorb.system.v1.HotspotConfig (triorb-system-core network.proto)
string ssid          # hotspot SSID
string password      # write-only: accepted in requests, never populated in responses
string band          # "2.4GHz" | "5GHz" ("" = backend default)
uint16 channel       # 0 = automatic
string ipv4_address  # gateway address of the hotspot subnet
bool enabled         # hotspot enabled
