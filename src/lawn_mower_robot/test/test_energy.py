import dataclasses
import math

import pytest

from lawn_mower_robot.energy import STANDARD_GRAVITY, EnergyIntegrator, SkidSteerEnergyModel


@pytest.fixture
def model(pioneer):
    return SkidSteerEnergyModel.from_config(pioneer)


def test_straight_closed_form(model, pioneer):
    m, mu_r, eta = pioneer.inertia.mass, pioneer.terrain.rolling_resistance, \
        pioneer.drive.drivetrain_efficiency
    e = model.straight(2.0, 0.35)
    assert e.turning == 0
    assert e.rolling == pytest.approx(mu_r * m * STANDARD_GRAVITY * 2.0)
    assert e.kinetic == pytest.approx(m * 0.35 ** 2 / 2)
    assert e.electrical == pytest.approx(e.mechanical / eta + e.auxiliary)


def test_in_place_turn_closed_form(model, pioneer):
    g, t = pioneer.geometry, pioneer.terrain
    weight = pioneer.inertia.mass * STANDARD_GRAVITY
    e = model.turn(math.pi / 2, 1.0)
    assert e.turning == pytest.approx(t.lateral_friction * weight * g.wheelbase / 2 * math.pi / 2)
    # Both sides roll chi B / 2 per radian.
    side = t.icr_expansion_factor * g.track_width / 2 * math.pi / 2
    assert e.rolling == pytest.approx(t.rolling_resistance * weight * side)


def test_turning_costs_more_than_driving_the_same_wheel_distance(model):
    assert model.turn(math.pi, 1.0).mechanical > model.straight(
        model.kinematics.effective_track_width * math.pi / 2, 0.3).mechanical


def test_energy_scales_with_config_symbols(model):
    heavy = dataclasses.replace(model, mass=2 * model.mass)
    assert heavy.turn(1.0, 1.0).turning == pytest.approx(2 * model.turn(1.0, 1.0).turning)
    lossy = dataclasses.replace(model, drivetrain_efficiency=model.drivetrain_efficiency / 2)
    assert lossy.straight(1.0, 0.3).electrical == pytest.approx(
        2 * model.straight(1.0, 0.3).electrical)


def test_auxiliary_power_is_charged_per_second(model):
    busy = dataclasses.replace(model, auxiliary_power=10.0)
    e = busy.straight(1.0, 0.5)
    assert e.auxiliary == pytest.approx(10.0 * 2.0)


def test_integrator_matches_closed_form(model):
    v, dt, n = 0.35, 0.01, 1000
    integrator = EnergyIntegrator(model)
    integrator.add(0.0, 0.0, 0.0)
    for i in range(1, n + 1):
        integrator.add(i * dt, v, 0.0)
    expected = model.straight(v * n * dt, v)
    # The first sample jumps from rest, so the trapezoid loses half a step of rolling.
    assert integrator.energy.electrical == pytest.approx(expected.electrical, rel=1e-2)
    assert integrator.energy.kinetic == pytest.approx(expected.kinetic)


def test_watt_hours(model):
    e = model.straight(10.0, 0.5)
    assert e.electrical_wh == pytest.approx(e.electrical / 3600)
