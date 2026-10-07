# Copyright 2026 TriOrb Inc.
# SPDX-License-Identifier: Apache-2.0

import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message

from triorb_snr_mux_interface.msg import SnrMuxFrame, SnrMuxStatus


@pytest.mark.parametrize('size', [0, 36, 255])
def test_frame_preserves_payload_and_receive_metadata(size):
    frame = SnrMuxFrame(
        stamp=Time(sec=42, nanosec=123456789),
        connection_generation=2**40,
        frame_id=0xA1 if size == 36 else 0xFE,
        sequence=255,
        payload=list(range(size)),
    )

    restored = deserialize_message(serialize_message(frame), SnrMuxFrame)

    assert restored == frame
    assert bytes(restored.payload) == bytes(range(size))


def test_payload_cannot_exceed_one_byte_length_field():
    with pytest.raises(AssertionError):
        SnrMuxFrame(payload=[0] * 256)


def test_zero_receive_time_is_distinct_from_no_received_frame():
    unseen = SnrMuxStatus(state=SnrMuxStatus.STATE_CONNECTED)
    received = SnrMuxStatus(
        state=SnrMuxStatus.STATE_CONNECTED,
        has_received_frame=True,
        last_receive_stamp=Time(sec=0, nanosec=0),
        receive_stale=True,
        crc_error_count=2**40,
        invalid_length_count=7,
        send_error_count=9,
        dropped_frame_count=11,
        reason='正常フレームの受信が途絶',
    )

    restored = deserialize_message(serialize_message(received), SnrMuxStatus)

    assert not unseen.has_received_frame
    assert restored.has_received_frame
    assert restored.last_receive_stamp == unseen.last_receive_stamp
    assert restored == received
