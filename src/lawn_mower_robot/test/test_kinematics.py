import math

import pytest

from lawn_mower_robot.kinematics import SkidSteerKinematics


@pytest.fixture
def kin(pioneer):
    return SkidSteerKinematics.from_config(pioneer)


@pytest.mark.parametrize('v, omega', [(0.3, 0.0), (0.0, 1.0), (0.2, -0.5), (-0.1, 0.3)])
def test_inverse_then_forward_is_identity(kin, v, omega):
    assert kin.forward(*kin.inverse(v, omega)) == pytest.approx((v, omega))


def test_straight_line_has_equal_wheel_speeds(kin):
    left, right = kin.inverse(0.4, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.4 / kin.effective_wheel_radius)


def test_expansion_factor_widens_the_track(pioneer, kin):
    chi = pioneer.terrain.icr_expansion_factor
    assert kin.effective_track_width == pytest.approx(chi * pioneer.geometry.track_width)
    # In-place turn: wheel speed is chi times the ideal differential-drive one.
    _, right = kin.inverse(0.0, 1.0)
    assert right * kin.effective_wheel_radius == pytest.approx(chi * pioneer.geometry.track_width / 2)


def test_saturate_respects_limits_and_keeps_curvature(kin):
    v, omega = kin.saturate(5.0, 5.0)
    assert abs(v) <= kin.max_linear_speed + 1e-9
    assert abs(omega) <= kin.max_angular_speed + 1e-9
    assert max(abs(w) for w in kin.inverse(v, omega)) <= kin.wheel_max_speed + 1e-9
    assert omega / v == pytest.approx(1.0)


def test_saturate_leaves_feasible_command(kin):
    assert kin.saturate(0.1, 0.1) == (0.1, 0.1)


def test_integrate_quarter_circle(kin):
    x, y, theta = kin.integrate((0.0, 0.0, 0.0), 0.5, 0.5, math.pi)
    assert (x, y, theta) == pytest.approx((1.0, 1.0, math.pi / 2))
