# Copyright 2023 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0
builtin_interfaces/Time stamp
# Nonempty ID unique to each client process start.
string<=64 source_session
uint64 event_id
# Uses AuxOutput.CUE_* constants.
uint8 sound_cue
uint8 volume
# Expiry is stamp + valid_for_ms, then tracked using the server's steady clock.
uint32 valid_for_ms
