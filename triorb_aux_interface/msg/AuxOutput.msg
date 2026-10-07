# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

# 継続出力の完全なsnapshot。部分更新ではない。
# 列挙値はAuxCommandConstantsの対応する定数を使う。
# stampは出力決定時刻。
builtin_interfaces/Time stamp
uint8 led_pattern
uint8 led_color
uint16 led_brightness
uint8 led_flash

# 最大４行、各行最大16 byte。ASCIIのみ、行要素内の「||」は禁止。
# 行の連結と符号化はbridgeが行う。ASCII・区切り制約はbridgeで検証する。
string<=16[<=4] lcd_lines

# SPEAKER_MODE_STOP/LOOP。STOP時はsound_cueを再生しない。
uint8 speaker_mode
uint8 sound_cue
uint8 speaker_volume
bool lamp_on

# led_brightnessは0〜4095、speaker_volumeは0〜62。
# 数値範囲・未対応の列挙値はbridgeで検証する。
