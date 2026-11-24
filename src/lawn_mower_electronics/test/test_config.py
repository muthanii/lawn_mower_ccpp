import os

import pytest
import yaml

from lawn_mower_electronics.config import (ElectronicsConfigError, parse_electronics_config,
                                           resolve_electronics_config)

from conftest import CONFIG_DIR


def raw():
    with open(os.path.join(CONFIG_DIR, 'faculty_robot.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_loads(electronics):
    assert electronics.name == 'faculty_robot'
    assert electronics.rc.channels.blade == 6
    assert electronics.sensors.imu.rate > 0
    assert not hasattr(electronics.sensors, 'gnss')


def test_missing_and_unknown_keys_are_listed_together():
    data = raw()
    del data['rc']['link_timeout']
    data['blade']['colour'] = 'red'
    with pytest.raises(ElectronicsConfigError) as e:
        parse_electronics_config(data)
    assert 'rc.link_timeout: missing' in str(e.value)
    assert 'blade.colour: unknown key' in str(e.value)


def test_failsafe_needs_one_value_per_channel():
    data = raw()
    data['rc']['failsafe'] = [1500]
    with pytest.raises(ElectronicsConfigError, match='rc.failsafe'):
        parse_electronics_config(data)


def test_channel_out_of_range():
    data = raw()
    data['rc']['channels']['mode'] = 9
    with pytest.raises(ElectronicsConfigError, match='rc.channels.mode'):
        parse_electronics_config(data)


def test_failsafe_stops_the_blade_and_selects_manual(electronics):
    rc, ctl = electronics.rc, electronics.controller
    assert rc.failsafe[rc.channels.blade - 1] <= ctl.switch_low
    assert rc.failsafe[rc.channels.mode - 1] <= ctl.switch_low
    assert rc.failsafe[rc.channels.left - 1] == rc.failsafe[rc.channels.right - 1] == rc.pulse_mid


def test_resolve_name_and_path():
    assert resolve_electronics_config('faculty_robot').endswith('faculty_robot.yaml')
    assert resolve_electronics_config('./x.yaml') == os.path.abspath('x.yaml')
