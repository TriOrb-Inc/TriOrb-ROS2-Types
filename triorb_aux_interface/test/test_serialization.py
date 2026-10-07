# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

import pytest
from builtin_interfaces.msg import Duration, Time
from rclpy.serialization import deserialize_message, serialize_message
from unique_identifier_msgs.msg import UUID

from triorb_aux_interface.msg import AuxCommandConstants, AuxOutput, AuxSoundOnce
from triorb_aux_interface.srv import PlayAuxSoundOnce, SetAuxOutput


def test_complete_output_with_maximum_lcd_and_levels_round_trips():
    output = AuxOutput(
        stamp=Time(sec=42, nanosec=123456789),
        led_pattern=AuxCommandConstants.LED_PATTERN_FORWARD,
        led_color=AuxCommandConstants.LED_COLOR_GREEN,
        led_brightness=4095,
        led_flash=AuxCommandConstants.LED_FLASH_VERY_FAST,
        lcd_lines=['0123456789ABCDEF'] * 4,
        speaker_mode=AuxCommandConstants.SPEAKER_MODE_LOOP,
        sound_cue=AuxCommandConstants.SOUND_CUE_MELODY,
        speaker_volume=62,
        lamp_on=True,
    )
    request = SetAuxOutput.Request(output=output)

    restored = deserialize_message(serialize_message(request), SetAuxOutput.Request)

    assert restored == request


@pytest.mark.parametrize('lines', [['x'] * 5, ['x' * 17]])
def test_lcd_declared_bounds_reject_oversized_values(lines):
    with pytest.raises(AssertionError):
        AuxOutput(lcd_lines=lines)


def test_default_output_requests_no_sound_or_light():
    output = AuxOutput()

    assert output.led_pattern == AuxCommandConstants.LED_PATTERN_OFF
    assert output.speaker_mode == AuxCommandConstants.SPEAKER_MODE_STOP
    assert output.sound_cue == AuxCommandConstants.SOUND_CUE_NONE
    assert not output.lamp_on


def test_sound_identity_and_expiry_survive_service_serialization():
    sound = AuxSoundOnce(
        stamp=Time(sec=123, nanosec=456),
        source_session=UUID(uuid=list(range(16))),
        event_id=2**40,
        sound_cue=AuxCommandConstants.SOUND_CUE_FINISH,
        volume=62,
        valid_for=Duration(sec=2, nanosec=500000000),
    )
    request = PlayAuxSoundOnce.Request(sound=sound)

    restored = deserialize_message(serialize_message(request), PlayAuxSoundOnce.Request)

    assert restored == request
    assert bytes(restored.sound.source_session.uuid) == bytes(range(16))


@pytest.mark.parametrize('service', [SetAuxOutput, PlayAuxSoundOnce])
def test_response_defaults_to_rejection(service):
    response = service.Response()

    assert not response.accepted
    assert response.result_code == service.Response.INVALID_REQUEST


@pytest.mark.parametrize('service', [SetAuxOutput, PlayAuxSoundOnce])
def test_rejection_preserves_reason_code_and_message(service):
    response = service.Response(
        accepted=False,
        result_code=service.Response.BUSY,
        message='送信queueが満杯',
    )

    restored = deserialize_message(serialize_message(response), service.Response)

    assert restored == response
