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

#==失敗の種類==
# Service の応答・Action の結果の `uint8 code` に入れる値。定数だけで項目は持たない。
# 名前と番号は gRPC の google/rpc/code.proto と同じ。
uint8 OK=0                    # 成功
uint8 CANCELLED=1             # 呼び出し元が取り消した
uint8 UNKNOWN=2               # 種類が分からない失敗
uint8 INVALID_ARGUMENT=3      # 要求の値が不正（状態によらない）
uint8 DEADLINE_EXCEEDED=4     # 時間内に終わらなかった
uint8 NOT_FOUND=5             # 対象が無い
uint8 ALREADY_EXISTS=6        # 作ろうとした対象が既にある
uint8 PERMISSION_DENIED=7     # 権限が無い
uint8 RESOURCE_EXHAUSTED=8    # 容量・数の上限に達した
uint8 FAILED_PRECONDITION=9   # 今の状態では実行できない
uint8 ABORTED=10              # 競合などで中断した
uint8 OUT_OF_RANGE=11         # 値が有効な範囲の外
uint8 UNIMPLEMENTED=12        # 未対応・未実装
uint8 INTERNAL=13             # 内部の異常
uint8 UNAVAILABLE=14          # 一時的に使えない（再試行で直りうる）
uint8 DATA_LOSS=15            # データが失われた・壊れた
uint8 UNAUTHENTICATED=16      # 認証されていない
