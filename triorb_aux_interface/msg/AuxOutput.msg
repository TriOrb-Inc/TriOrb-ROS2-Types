# Copyright 2023 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0
uint8 LED_OFF=0
uint8 LED_SOLID=1
uint8 LED_FLASH=2
uint8 LED_FORWARD=3
uint8 LED_FORWARD_RIGHT=4
uint8 LED_RIGHT=5
uint8 LED_BACKWARD_RIGHT=6
uint8 LED_BACKWARD=7
uint8 LED_BACKWARD_LEFT=8
uint8 LED_LEFT=9
uint8 LED_FORWARD_LEFT=10
uint8 LED_CLOCKWISE=11
uint8 LED_COUNTERCLOCKWISE=12

uint8 COLOR_WHITE=0
uint8 COLOR_BLUE=1
uint8 COLOR_GREEN=2
uint8 COLOR_RED=3
uint8 COLOR_CYAN=4
uint8 COLOR_MAGENTA=5
uint8 COLOR_YELLOW=6
uint8 COLOR_BLACK=7

uint8 FLASH_SLOW=0
uint8 FLASH_MEDIUM=1
uint8 FLASH_FAST=2
uint8 FLASH_VERY_FAST=3
uint8 SPEAKER_LOOP=0
uint8 SPEAKER_STOP=2

# Semantic cue IDs. The bridge maps these to robot-specific playlist numbers.
uint8 CUE_TEST=0
uint8 CUE_START=1
uint8 CUE_MELODY=2
uint8 CUE_EMERGENCY=3
uint8 CUE_ERROR=4
uint8 CUE_LOST=5
uint8 CUE_MOTOR_CONNECTION_ERROR=6
uint8 CUE_LOST_RECOVERED=7
uint8 CUE_EMERGENCY_PRESSED=8
uint8 CUE_CHARGING=9
uint8 CUE_BATTERY_LOW=10
uint8 CUE_TAG_SLAM=11
uint8 CUE_NAV_FAILED=12
uint8 CUE_STARTUP=13
uint8 CUE_FINISH=14

# Request creation time; refresh this timestamp for each heartbeat.
builtin_interfaces/Time stamp
uint8 led_pattern
uint8 led_color
uint16 led_brightness
uint8 led_flash
# Printable ASCII; no embedded ||. Joined payload must fit in 64 bytes.
string<=16[<=4] lcd_lines
uint8 speaker_mode
uint8 sound_cue
uint8 speaker_volume
bool lamp_on
