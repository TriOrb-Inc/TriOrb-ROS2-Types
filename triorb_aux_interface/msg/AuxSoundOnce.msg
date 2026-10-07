# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

# 単発音声イベントの発生時刻。
builtin_interfaces/Time stamp

# 発行元の起動ごとに新しい非ゼロUUID。同一起動中は保持する。
unique_identifier_msgs/UUID source_session
# session内でイベントごとに一意。同一要求の再配送時は変更しない。
uint64 event_id

# AuxCommandConstants.SOUND_CUE_*。NONEは単発再生要求に使わない。
uint8 sound_cue
# 0〜62。bridgeで範囲を検証する。
uint8 volume

# stampからの有効期間。正数で、無期限を表す特別値は設けない。
# 受信後の残り時間の監視には単調増加時計を使う。
builtin_interfaces/Duration valid_for
