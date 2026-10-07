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

uint8 STATE_INIT=0
uint8 STATE_CONNECTING=1
uint8 STATE_CONNECTED=2
uint8 STATE_RETRY_WAIT=3
uint8 STATE_STOPPING=4
uint8 STATE_ERROR=5

# 状態の作成時刻。stateは上記STATE_*、診断のstate文字列と同じ内部状態を表す。
builtin_interfaces/Time stamp
uint8 state

# 現在の接続での正常フレーム受信の有無。再接続時にfalseへ戻す。
# falseのときlast_receive_stampは無効。時刻0だけで未受信を判定しない。
bool has_received_frame
builtin_interfaces/Time last_receive_stamp

# 正常フレームの未受信／途絶が設定閾値に達したか。単調増加時計で判定する。
# ポートの接続状態とは独立。watchdog専用の監視ではない。
bool receive_stale
