# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

# CRCとフレーム長を検証済みのSNR-MUX受信フレーム。
# stampは受信完了時刻であり、センサ計測時刻ではない。
builtin_interfaces/Time stamp

# bridgeプロセス内の接続世代。再接続ごとに変わる。
# プロセス再起動を跨ぐ一意性や、再起動の検出は保証しない。
uint64 connection_generation

# 未知のIDも運搬する。sequenceは他IDと共有する0〜255の連番。
uint8 frame_id
uint8 sequence

# ヘッダ・CRCを含まない。LENはこの配列の長さ。IMUの36 byte検証はparserが行う。
uint8[<=255] payload
