import math

import pytest

from lawn_mower_navigation.localization import wrap
from lawn_mower_navigation.tracker import ALIGN, DRIVE, PathTracker, Segment, TrackerParams
from lawn_mower_robot.kinematics import SkidSteerKinematics

DT = 0.05
PARAMS = TrackerParams(cruise_speed=0.6, turn_rate=0.6, linear_deceleration=0.4,
                       angular_deceleration=0.8)
LANES = [Segment((0, 0), (5, 0), True, True), Segment((5, 0), (5, 0.5), True, True),
         Segment((5, 0.5), (0, 0.5), True, True)]


def simulate(tracker, pose, robot, seconds=120.0, lag=0.15, blade=True):
    """Kinematic robot with a first-order lag on the commanded velocities."""
    kin = SkidSteerKinematics.from_config(robot)
    v = w = 0.0
    trail = [pose]
    for _ in range(int(seconds / DT)):
        cmd = tracker.update(pose, blade)
        if cmd.done:
            break
        v += (cmd.v - v) * DT / lag
        w += (cmd.omega - w) * DT / lag
        pose = kin.integrate(pose, v, w, DT)
        trail.append(pose)
    return trail


def cross_track(pose, seg):
    c, s = math.cos(seg.heading), math.sin(seg.heading)
    return abs(-(pose[0] - seg.start[0]) * s + (pose[1] - seg.start[1]) * c)


def test_follows_two_lanes_and_stops_at_the_end(robot):
    tracker = PathTracker(LANES, PARAMS)
    trail = simulate(tracker, (0.0, 0.0, 0.0), robot)
    assert tracker.done
    assert math.dist(trail[-1][:2], LANES[-1].end) < 0.06
    assert max(min(cross_track(p, s) for s in LANES) for p in trail) < 0.05


def test_turns_in_place_before_driving(robot):
    tracker = PathTracker(LANES, PARAMS)
    cmd = tracker.update((0.0, 0.0, math.radians(90)))
    assert tracker.phase == ALIGN and cmd.v == 0 and cmd.omega < 0   # turn right, to heading 0
    cmd = tracker.update((0.0, 0.0, 0.0))
    assert tracker.phase == DRIVE and cmd.v > 0


def test_steers_back_from_a_lateral_offset(robot):
    tracker = PathTracker([Segment((0, 0), (10, 0), True, True)], PARAMS)
    trail = simulate(tracker, (0.0, 0.25, 0.0), robot)
    assert abs(trail[-1][1]) < 0.03
    assert abs(wrap(trail[-1][2])) < math.radians(3)
    assert max(abs(p[1]) for p in trail) <= 0.25 + 1e-9    # no overshoot beyond the start


def test_waits_for_the_blade_on_a_mowing_segment(robot):
    tracker = PathTracker(LANES, PARAMS)
    cmd = tracker.update((0.0, 0.0, 0.0), blade_running=False)
    assert (cmd.v, cmd.omega, cmd.blade) == (0.0, 0.0, True)
    assert tracker.update((0.0, 0.0, 0.0), blade_running=True).v > 0


def test_blade_is_off_on_a_long_move_and_does_not_wait(robot):
    tracker = PathTracker([Segment((0, 0), (5, 0), False)], PARAMS)
    cmd = tracker.update((0.0, 0.0, 0.0), blade_running=False)
    assert cmd.v > 0 and not cmd.blade


def test_slows_before_the_end_of_a_segment(robot):
    tracker = PathTracker([Segment((0, 0), (5, 0), False)], PARAMS)
    assert tracker.update((2.0, 0.0, 0.0)).v == pytest.approx(PARAMS.cruise_speed)
    assert tracker.update((4.9, 0.0, 0.0)).v < PARAMS.cruise_speed / 2


def test_hold_makes_it_turn_onto_the_segment_again(robot):
    tracker = PathTracker(LANES, PARAMS)
    tracker.update((0.0, 0.0, 0.0))
    assert tracker.phase == DRIVE
    tracker.hold()
    cmd = tracker.update((1.0, 0.0, math.radians(20)))    # pushed off by hand
    assert cmd.v == 0 and cmd.omega < 0


def test_a_large_heading_error_stops_and_turns(robot):
    tracker = PathTracker(LANES, PARAMS)
    tracker.update((0.0, 0.0, 0.0))
    cmd = tracker.update((1.0, 0.0, math.radians(45)))
    assert cmd.v == 0 and cmd.omega < 0


def test_empty_path_is_done():
    cmd = PathTracker([], PARAMS).update((0.0, 0.0, 0.0))
    assert cmd.done and not cmd.blade
