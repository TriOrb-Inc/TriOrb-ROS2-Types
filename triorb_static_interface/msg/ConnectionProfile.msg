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

#==NetworkManager connection profile==
# Mirrors triorb.system.v1.ConnectionProfile (triorb-system-core network.proto)
string id         # connection id (NetworkManager "NAME")
string type       # e.g. "802-3-ethernet", "802-11-wireless"
string device     # bound device, "" if none
bool active       # currently active
bool autoconnect  # autoconnect enabled
