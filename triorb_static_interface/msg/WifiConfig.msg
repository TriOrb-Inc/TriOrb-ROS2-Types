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

#==Wi-Fi(station)接続設定==
# Mirrors triorb.system.v1.WifiConfig (triorb-system-core network.proto)
string ssid          # SSID to connect to
string password      # write-only: accepted in requests, never populated in responses
bool autoconnect     # autoconnect enabled
string ipv4_method   # "auto" (DHCP) | "manual" (static)
string ipv4_address  # static only
uint8 ipv4_prefix    # static only, network prefix length
string ipv4_gateway  # static only
string[] dns         # static only
bool connected       # connection state (read-only in config set; toggled by /network/wifi/connect/toggle)
