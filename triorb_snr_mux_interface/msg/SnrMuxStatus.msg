# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

uint8 STATE_INIT=0
uint8 STATE_CONNECTING=1
uint8 STATE_CONNECTED=2
uint8 STATE_RETRY_WAIT=3
uint8 STATE_STOPPING=4
uint8 STATE_ERROR=5

# 状態の作成時刻。stateは上記STATE_*、診断のstate文字列と同じ内部状態を表す。
builtin_interfaces/Time stamp
uint8 state
uint64 connection_generation

# 現接続世代での正常フレーム受信の有無。再接続時にfalseへ戻す。
# falseのときlast_receive_stampは無効。時刻0だけで未受信を判定しない。
bool has_received_frame
builtin_interfaces/Time last_receive_stamp

# 正常フレームの未受信／途絶が設定閾値に達したか。単調増加時計で判定する。
# ポートの接続状態とは独立。watchdog専用の監視ではない。
bool receive_stale

# bridgeプロセスの起動からの累積件数。再接続ではリセットしない。
uint64 crc_error_count
uint64 invalid_length_count
uint64 send_error_count
uint64 dropped_frame_count

# 人向けの理由。機械的な分岐条件には使わない。
string reason
