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

#==ネットワークインターフェース情報==
# Mirrors triorb.system.v1.NetworkInterface (triorb-system-core hostinfo.proto)
string name                  # interface name, e.g. "eth0"
bool is_up                   # administrative/link state (IFF_UP)
bool is_loopback             # IFF_LOOPBACK
IpAddress[] ipv4_addresses   # assigned IPv4 addresses
IpAddress[] ipv6_addresses   # assigned IPv6 addresses
