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

#==ロボット → FMS: コマンドへの応答==
# 1 コマンドにつき、受理時(accepted)と完了時(succeeded / failed)に出る。拒否は rejected の 1 通だけ。
# stop は accepted の後、停止を観測したら confirmed、観測できなければ failed
builtin_interfaces/Time stamp
string request_id                   # コマンドの request_id
string PHASE_ACCEPTED="accepted"    # 受理した。以後 succeeded / failed / confirmed のどれかが来る
string PHASE_REJECTED="rejected"    # 検証で拒否した。これで終わり
string PHASE_SUCCEEDED="succeeded"  # 完了。ROS 2 側の呼び出しが成功した
string PHASE_FAILED="failed"        # 完了。失敗または deadline 超過
string PHASE_CONFIRMED="confirmed"  # 完了。stop のみ。停止を観測した
string phase                        # PHASE_*
string message                      # 拒否・失敗の理由
