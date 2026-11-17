import math

import numpy as np
import pytest

from lawn_mower_navigation.localization import PoseFilter, SpeedModel, wrap

GNSS_STDDEV = 0.02
DT = 0.01


def drive(filt, truth, v, omega, seconds, rng, speed_error=0.0, fix_every=10):
    """Drive the true pose and feed the filter a noisy gyro and GNSS fixes."""
    x, y, th = truth
    for i in range(int(seconds / DT)):
        x += v * DT * math.cos(th)
        y += v * DT * math.sin(th)
        th += omega * DT
        filt.predict(v + speed_error, omega + rng.normal(0, 0.005), DT)
        if i % fix_every == 0:
            filt.correct(x + rng.normal(0, GNSS_STDDEV), y + rng.normal(0, GNSS_STDDEV), GNSS_STDDEV)
    return x, y, th


def new_filter(heading=0.0):
    filt = PoseFilter(speed_stddev=0.2, yaw_rate_stddev=0.02, heading_stddev=0.5)
    filt.reset(0.0, 0.0, heading, GNSS_STDDEV)
    return filt


def test_not_ready_before_the_first_fix():
    filt = PoseFilter(0.2, 0.02, 0.5)
    assert not filt.ready
    filt.predict(1.0, 0.0, DT)   # ignored
    assert not filt.ready


def test_tracks_a_straight_line_and_a_turn():
    rng = np.random.default_rng(0)
    filt = new_filter()
    truth = drive(filt, (0.0, 0.0, 0.0), 0.6, 0.0, 10.0, rng)
    truth = drive(filt, truth, 0.0, 0.6, 2.0, rng)
    truth = drive(filt, truth, 0.6, 0.0, 10.0, rng)
    x, y, th = filt.pose
    assert math.hypot(x - truth[0], y - truth[1]) < 0.03
    assert abs(wrap(th - truth[2])) < math.radians(2)


def test_a_wrong_first_heading_is_corrected_by_driving():
    rng = np.random.default_rng(1)
    filt = new_filter(heading=math.radians(25))
    truth = drive(filt, (0.0, 0.0, 0.0), 0.6, 0.0, 10.0, rng)
    assert abs(wrap(filt.pose[2] - truth[2])) < math.radians(2)


def test_a_biased_speed_is_held_by_the_fixes():
    rng = np.random.default_rng(2)
    filt = new_filter()
    truth = drive(filt, (0.0, 0.0, 0.0), 0.6, 0.0, 20.0, rng, speed_error=0.1)
    assert math.hypot(filt.pose[0] - truth[0], filt.pose[1] - truth[1]) < 0.05


def test_heading_stays_wrapped():
    filt = new_filter(heading=3.1)
    filt.predict(0.0, 1.0, 0.5)
    assert -math.pi <= filt.pose[2] <= math.pi


def test_speed_model_ramps_at_the_acceleration_limit():
    model = SpeedModel(time_constant=0.15, max_acceleration=0.5)
    speeds = [model.update(0.6, DT) for _ in range(int(3.0 / DT))]
    assert speeds[int(0.5 / DT)] <= 0.5 * (0.5 + 2 * DT)   # no faster than a t
    assert speeds[-1] == pytest.approx(0.6, abs=1e-3)
    assert all(b >= a for a, b in zip(speeds, speeds[1:]))
    assert model.update(0.6, 0.0) == speeds[-1]


def test_speed_model_brakes_too():
    model = SpeedModel(time_constant=0.15, max_acceleration=0.5)
    for _ in range(int(3.0 / DT)):
        model.update(0.6, DT)
    after = [model.update(0.0, DT) for _ in range(int(3.0 / DT))]
    assert after[int(0.5 / DT)] >= 0.6 - 0.5 * (0.5 + 2 * DT)
    assert after[-1] == pytest.approx(0.0, abs=1e-3)
