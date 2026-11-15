import pytest

rclpy = pytest.importorskip('rclpy')

from lawn_mower_electronics.transmitter_node import build_frame  # noqa: E402


def test_sticks_are_mixed_to_the_two_sides(electronics):
    rc = electronics.rc
    frame = build_frame(rc, 'manual', False, 0.5, 0.0)
    assert frame[rc.channels.left - 1] == frame[rc.channels.right - 1] > rc.pulse_mid
    frame = build_frame(rc, 'manual', False, 0.0, 0.5)   # turn left in place
    assert frame[rc.channels.left - 1] < rc.pulse_mid < frame[rc.channels.right - 1]


def test_switches(electronics):
    rc, ctl = electronics.rc, electronics.controller
    frame = build_frame(rc, 'auto', True, 0.0, 0.0)
    assert frame[rc.channels.mode - 1] >= ctl.switch_high
    assert frame[rc.channels.blade - 1] >= ctl.switch_high
    frame = build_frame(rc, 'manual', False, 0.0, 0.0)
    assert frame[rc.channels.mode - 1] <= ctl.switch_low
    assert frame[rc.channels.blade - 1] <= ctl.switch_low
