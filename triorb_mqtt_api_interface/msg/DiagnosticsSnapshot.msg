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

#==ロボット → FMS: 診断の全文(DiagnosticsRequest への応答)==
# robot/state/get の応答(DiagnosticArray)をそのまま写す
string request_id              # 要求の request_id
builtin_interfaces/Time stamp  # 集約結果の header.stamp
diagnostic_msgs/DiagnosticStatus[] status
