import pytest

from lawn_mower_electronics.battery import SECONDS_PER_HOUR, Battery
from lawn_mower_electronics.blade import Blade
from lawn_mower_electronics.motor_driver import MotorDriver

LONG = 100.0  # s, many time constants


@pytest.fixture
def driver(electronics, robot):
    return MotorDriver(electronics, robot.drive.wheel_max_speed)


def test_driver_is_off_inside_the_deadband(driver, electronics):
    rc = electronics.rc
    driver.step(0.0)
    driver.set_pulses(LONG, rc.pulse_mid + electronics.motor_driver.deadband, rc.pulse_mid)
    assert driver.step(LONG) == (0.0, 0.0)


def test_driver_reaches_the_no_load_speed_at_full_pulse(driver, electronics, robot):
    rc = electronics.rc
    driver.step(0.0)
    driver.set_pulses(LONG, rc.pulse_max, rc.pulse_min)
    left, right = driver.step(LONG)
    assert left == pytest.approx(robot.drive.wheel_max_speed)
    assert right == pytest.approx(-robot.drive.wheel_max_speed)


def test_driver_lags_by_its_time_constant(driver, electronics, robot):
    tau = electronics.motor_driver.time_constant
    driver.step(0.0)
    driver.set_pulses(0.0, electronics.rc.pulse_max, electronics.rc.pulse_max)
    left, _ = driver.step(tau)
    assert left == pytest.approx(0.632 * robot.drive.wheel_max_speed, rel=0.01)


def test_driver_stops_without_pulses(driver, electronics):
    driver.step(0.0)
    driver.set_pulses(0.0, electronics.rc.pulse_max, electronics.rc.pulse_max)
    driver.step(0.2)
    left, right = driver.step(LONG)   # no pulse since t = 0
    assert abs(left) < 1e-6 and abs(right) < 1e-6


def test_driver_stops_without_power(driver, electronics):
    driver.step(0.0)
    driver.set_pulses(LONG, electronics.rc.pulse_max, electronics.rc.pulse_max)
    left, right = driver.step(LONG, powered=False)
    assert (left, right) == (0.0, 0.0)


def test_blade_spins_up_then_cuts(electronics, robot):
    blade = Blade(electronics, robot.mower.blade_power)
    blade.step(0.0, True)
    assert not blade.running
    blade.step(LONG, True)
    assert blade.running
    assert blade.speed == pytest.approx(electronics.blade.rated_speed)
    assert blade.power() == pytest.approx(robot.mower.blade_power)
    assert blade.power(cutting=False) == pytest.approx(
        robot.mower.blade_power * electronics.blade.idle_power_fraction)


def test_blade_stops_when_the_relay_opens_or_power_goes(electronics, robot):
    blade = Blade(electronics, robot.mower.blade_power)
    blade.step(0.0, True)
    blade.step(LONG, True)
    blade.step(2 * LONG, False)
    assert not blade.running and blade.power() < 1e-6
    blade.step(3 * LONG, True, powered=False)
    assert not blade.running


def test_robot_without_a_blade_draws_nothing(electronics):
    blade = Blade(electronics, None)
    blade.step(0.0, True)
    blade.step(LONG, True)
    assert blade.power() == 0.0


def test_battery_counts_energy_by_consumer(electronics, robot):
    battery = Battery.from_configs(robot, electronics)
    battery.step(0.0, {'traction': 100.0, 'blade': 500.0})
    battery.step(SECONDS_PER_HOUR, {'traction': 100.0, 'blade': 500.0})
    assert battery.energy['traction'] == pytest.approx(100.0 * SECONDS_PER_HOUR)
    assert battery.energy['blade'] == pytest.approx(500.0 * SECONDS_PER_HOUR)
    used = 600.0 / (robot.battery.voltage * robot.battery.capacity)
    assert battery.state_of_charge == pytest.approx(1 - used)


def test_battery_voltage_sags_under_load(electronics, robot):
    battery = Battery.from_configs(robot, electronics)
    assert battery.step(0.0, {}) == pytest.approx(robot.battery.voltage)
    loaded = battery.step(1.0, {'blade': 500.0})
    assert loaded < robot.battery.voltage
    assert loaded * battery.current == pytest.approx(500.0)
    assert not battery.empty


def test_battery_runs_empty(electronics, robot):
    battery = Battery.from_configs(robot, electronics)
    battery.step(0.0, {'blade': 500.0})
    battery.step(10 * SECONDS_PER_HOUR, {'blade': 500.0})
    assert battery.state_of_charge == 0.0 and battery.empty


def test_power_monitor_matches_the_energy_model(electronics, robot):
    from lawn_mower_electronics.battery import PowerMonitor
    from lawn_mower_robot.energy import SkidSteerEnergyModel
    model = SkidSteerEnergyModel.from_config(robot)
    battery = Battery.from_configs(robot, electronics)
    monitor = PowerMonitor(model, battery)
    v, duration, steps = 0.6, 10.0, 500
    monitor.update(0.0, 0.0, 0.0, 0.0)
    for i in range(1, steps + 1):
        monitor.update(i * duration / steps, v, 0.0, 0.0)
    expected = model.straight(v * duration, v)
    traction = battery.energy[PowerMonitor.TRACTION]
    assert traction == pytest.approx(expected.mechanical / expected.efficiency, rel=0.01)
    assert battery.energy[PowerMonitor.AUXILIARY] == pytest.approx(expected.auxiliary, rel=0.01)
    assert battery.energy[PowerMonitor.BLADE] == 0.0
