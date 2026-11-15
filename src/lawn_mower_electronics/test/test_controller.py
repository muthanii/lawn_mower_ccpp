import pytest

from lawn_mower_electronics.controller import AUTO, ESTOP, FAILSAFE, MANUAL, Controller
from lawn_mower_electronics.motor_driver import MotorDriver

from conftest import frame


@pytest.fixture
def controller(electronics, kinematics):
    return Controller(electronics, kinematics)


def neutral(out, electronics):
    mid = electronics.rc.pulse_mid
    return out.left == mid and out.right == mid and not out.blade


def test_manual_passes_the_sticks_and_the_blade_switch(controller, electronics):
    out = controller.step(0.0, frame(electronics, left=1800, right=1200, blade=2000), False, False)
    assert (out.state, out.left, out.right, out.blade) == (MANUAL, 1800, 1200, True)


def test_manual_ignores_the_companion(controller, electronics):
    controller.set_command(0.0, 1.0, 0.0)
    controller.set_blade_request(True)
    out = controller.step(0.0, frame(electronics), False, False)
    assert out.state == MANUAL and neutral(out, electronics)


def test_auto_turns_a_velocity_command_into_pulses(controller, electronics, kinematics):
    v, omega = 0.6, 0.3
    controller.set_command(0.0, v, omega)
    out = controller.step(0.0, frame(electronics, mode=2000), False, False)
    assert out.state == AUTO
    driver = MotorDriver(electronics, kinematics.wheel_max_speed)
    left, right = driver.target(out.left), driver.target(out.right)
    got_v, got_omega = kinematics.forward(left, right)
    assert got_v == pytest.approx(v, abs=0.01) and got_omega == pytest.approx(omega, abs=0.02)


def test_auto_ignores_the_sticks(controller, electronics):
    controller.set_command(0.0, 0.0, 0.0)
    out = controller.step(0.0, frame(electronics, mode=2000, left=2000, right=2000), False, False)
    assert out.state == AUTO and neutral(out, electronics)


def test_auto_blade_needs_the_request_and_the_operator_switch(controller, electronics):
    controller.set_command(0.0, 0.5, 0.0)
    controller.set_blade_request(True)
    assert not controller.step(0.0, frame(electronics, mode=2000), False, False).blade
    assert controller.step(0.0, frame(electronics, mode=2000, blade=2000), False, False).blade
    controller.set_blade_request(False)
    assert not controller.step(0.0, frame(electronics, mode=2000, blade=2000), False, False).blade


def test_auto_stops_when_the_companion_goes_silent(controller, electronics):
    controller.set_command(0.0, 0.5, 0.0)
    controller.set_blade_request(True)
    late = electronics.controller.command_timeout * 2
    out = controller.step(late, frame(electronics, mode=2000, blade=2000), False, False)
    assert out.state == AUTO and neutral(out, electronics)


def test_auto_without_any_command_is_neutral(controller, electronics):
    out = controller.step(0.0, frame(electronics, mode=2000, blade=2000), False, False)
    assert neutral(out, electronics)


def test_operator_takes_over_by_flipping_the_mode_switch(controller, electronics):
    controller.set_command(0.0, 0.5, 0.0)
    assert controller.step(0.0, frame(electronics, mode=2000), False, False).state == AUTO
    out = controller.step(0.02, frame(electronics, mode=1000, left=1400, right=1400), False, False)
    assert (out.state, out.left, out.right) == (MANUAL, 1400, 1400)


def test_lost_radio_link_stops_everything_in_auto(controller, electronics):
    controller.set_command(0.0, 0.5, 0.0)
    controller.set_blade_request(True)
    controller.step(0.0, frame(electronics, mode=2000, blade=2000), False, False)
    out = controller.step(0.02, electronics.rc.failsafe, True, False)
    assert out.state == FAILSAFE and neutral(out, electronics)


def test_auto_resumes_when_the_link_returns(controller, electronics):
    controller.step(0.0, frame(electronics, mode=2000), False, False)
    controller.step(0.02, electronics.rc.failsafe, True, False)
    controller.set_command(0.04, 0.5, 0.0)
    out = controller.step(0.04, frame(electronics, mode=2000), False, False)
    assert out.state == AUTO and out.left > electronics.rc.pulse_mid


def test_estop_latches_until_the_operator_selects_manual(controller, electronics):
    controller.set_command(0.0, 0.5, 0.0)
    auto = frame(electronics, mode=2000, blade=2000)
    out = controller.step(0.0, auto, False, True)
    assert out.state == ESTOP and neutral(out, electronics)
    controller.set_command(0.02, 0.5, 0.0)
    assert controller.step(0.02, auto, False, False).state == ESTOP    # released, still latched
    assert controller.step(0.04, frame(electronics, mode=1000), False, False).state == MANUAL
    controller.set_command(0.06, 0.5, 0.0)
    assert controller.step(0.06, auto, False, False).state == AUTO


def test_estop_beats_manual(controller, electronics):
    out = controller.step(0.0, frame(electronics, left=2000, right=2000, blade=2000), False, True)
    assert out.state == ESTOP and neutral(out, electronics)


def test_commands_above_the_limits_are_scaled_not_clipped(controller, electronics, kinematics):
    left, right = controller.twist_to_pulses(10 * kinematics.max_linear_speed, 0.0)
    assert left == right <= electronics.rc.pulse_max


def test_small_commands_clear_the_driver_deadband(controller, electronics, kinematics):
    driver = MotorDriver(electronics, kinematics.wheel_max_speed)
    left, right = controller.twist_to_pulses(0.02, 0.0)
    assert driver.target(left) == pytest.approx(0.02 / kinematics.effective_wheel_radius, rel=0.2)
    assert controller.twist_to_pulses(0.0, 0.0) == (electronics.rc.pulse_mid,) * 2


def test_bumper_stops_automatic_mode_and_latches(controller, electronics):
    auto = frame(electronics, mode=2000, blade=2000)
    controller.set_command(0.0, 0.5, 0.0)
    controller.set_blade_request(True)
    out = controller.step(0.0, auto, False, False, bumper=True)
    assert out.state == ESTOP and neutral(out, electronics)
    controller.set_command(0.02, 0.5, 0.0)
    assert controller.step(0.02, auto, False, False, bumper=False).state == ESTOP


def test_operator_can_drive_off_the_obstacle_in_manual(controller, electronics):
    auto = frame(electronics, mode=2000)
    controller.step(0.0, auto, False, False, bumper=True)
    back = frame(electronics, mode=1000, left=1300, right=1300)
    out = controller.step(0.02, back, False, False, bumper=True)    # still touching
    assert (out.state, out.left, out.right) == (MANUAL, 1300, 1300)
    controller.set_command(0.04, 0.5, 0.0)
    assert controller.step(0.04, auto, False, False, bumper=False).state == AUTO


def test_bumper_does_not_stop_manual_driving(controller, electronics):
    out = controller.step(0.0, frame(electronics, left=1800, right=1800), False, False, bumper=True)
    assert out.state == MANUAL and out.left == 1800
