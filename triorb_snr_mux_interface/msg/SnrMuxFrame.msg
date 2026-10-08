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

# CRCとフレーム長を検証済みのSNR-MUX受信フレーム。
# stampは受信完了時刻であり、センサ計測時刻ではない。
builtin_interfaces/Time stamp

# bridgeプロセス内の接続世代。再接続ごとに変わる。
# プロセス再起動を跨ぐ一意性や、再起動の検出は保証しない。
uint64 connection_generation

# 未知のIDも運搬する。sequenceは他IDと共有する0〜255の連番。
uint8 msg_frame_id
uint8 sequence

# ヘッダ・CRCを含まない。LENはこの配列の長さ。IMUの36 byte検証はparserが行う。
uint8[<=255] payload
