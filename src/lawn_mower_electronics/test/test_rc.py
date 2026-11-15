import pytest

from lawn_mower_electronics.rc import Receiver, Switch, pulse_to_unit, unit_to_pulse

from conftest import frame


def test_pulse_and_unit_are_inverse(electronics):
    rc = electronics.rc
    assert pulse_to_unit(rc.pulse_min, rc) == -1
    assert pulse_to_unit(rc.pulse_mid, rc) == 0
    assert pulse_to_unit(rc.pulse_max, rc) == 1
    for unit in (-1, -0.5, 0, 0.25, 1):
        assert pulse_to_unit(unit_to_pulse(unit, rc), rc) == pytest.approx(unit, abs=0.01)


def test_pulses_are_clamped(electronics):
    rc = electronics.rc
    assert unit_to_pulse(3, rc) == rc.pulse_max
    assert pulse_to_unit(rc.pulse_max + 400, rc) == 1


def test_switch_has_hysteresis(electronics):
    ctl = electronics.controller
    switch = Switch(ctl.switch_low, ctl.switch_high)
    assert not switch.update(electronics.rc.pulse_mid)
    assert switch.update(ctl.switch_high)
    assert switch.update(electronics.rc.pulse_mid)      # between the thresholds: stays on
    assert not switch.update(ctl.switch_low)


def test_receiver_repeats_the_last_frame(electronics):
    receiver = Receiver(electronics.rc)
    sent = frame(electronics, left=1700, right=1300)
    receiver.feed(0.0, sent)
    channels, failsafe = receiver.output(electronics.rc.link_timeout / 2)
    assert channels == sent and not failsafe


def test_receiver_goes_to_failsafe_without_frames(electronics):
    receiver = Receiver(electronics.rc)
    assert receiver.output(0.0) == (electronics.rc.failsafe, True)   # never linked
    receiver.feed(0.0, frame(electronics, left=2000, right=2000, blade=2000))
    channels, failsafe = receiver.output(electronics.rc.link_timeout * 2)
    assert failsafe and channels == electronics.rc.failsafe


def test_receiver_recovers_when_the_link_returns(electronics):
    receiver = Receiver(electronics.rc)
    receiver.feed(0.0, frame(electronics))
    late = electronics.rc.link_timeout * 3
    assert receiver.output(late)[1]
    receiver.feed(late, frame(electronics))
    assert not receiver.output(late)[1]


def test_receiver_rejects_a_frame_of_the_wrong_size(electronics):
    with pytest.raises(ValueError):
        Receiver(electronics.rc).feed(0.0, [1500])
