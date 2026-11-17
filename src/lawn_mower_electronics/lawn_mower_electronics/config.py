"""Load an electronics config (config/<name>.yaml).

The file describes the radio, the controller, the motor driver, the blade,
the battery and the sensors as the simulation models them. The loader checks
that every key the models read is there, and nothing else.
"""

import os
from types import SimpleNamespace

import yaml

KEYS = {
    'rc': ('frame_period', 'pulse_min', 'pulse_mid', 'pulse_max', 'channel_count',
           'channels', 'failsafe', 'link_timeout'),
    'controller': ('rate', 'switch_high', 'switch_low', 'command_timeout'),
    'motor_driver': ('deadband', 'time_constant', 'signal_timeout'),
    'blade': ('rated_speed', 'time_constant', 'running_fraction', 'idle_power_fraction'),
    'battery': ('internal_resistance', 'cutoff_voltage'),
    'bumper': ('hold_time',),
    'sensors': ('gnss', 'imu'),
}
CHANNELS = ('right', 'left', 'mode', 'blade')
SENSOR_KEYS = {'gnss': ('rate', 'horizontal_stddev', 'vertical_stddev'),
               'imu': ('rate', 'gyro_stddev', 'accel_stddev')}


class ElectronicsConfigError(ValueError):
    """The electronics config file is missing keys or has invalid values."""


def _namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: _namespace(v) for k, v in value.items()})
    return value


def _check_keys(where, section, keys, problems):
    if not isinstance(section, dict):
        problems.append(f'{where}: missing section')
        return
    for key in keys:
        if key not in section:
            problems.append(f'{where}.{key}: missing')
    for key in sorted(set(section) - set(keys)):
        problems.append(f'{where}.{key}: unknown key')


def parse_electronics_config(data, path='<memory>'):
    problems = []
    if not isinstance(data, dict):
        raise ElectronicsConfigError(f'{path}: not a YAML mapping')
    for group, keys in KEYS.items():
        _check_keys(group, data.get(group), keys, problems)
    rc = data.get('rc') or {}
    _check_keys('rc.channels', rc.get('channels'), CHANNELS, problems)
    for sensor, keys in SENSOR_KEYS.items():
        _check_keys(f'sensors.{sensor}', (data.get('sensors') or {}).get(sensor), keys, problems)
    if not problems:
        count = rc['channel_count']
        if len(rc['failsafe']) != count:
            problems.append(f'rc.failsafe: needs {count} values, one for each channel')
        for name, number in rc['channels'].items():
            if not 1 <= number <= count:
                problems.append(f'rc.channels.{name}: channel {number} is not in 1..{count}')
        if not rc['pulse_min'] < rc['pulse_mid'] < rc['pulse_max']:
            problems.append('rc: needs pulse_min < pulse_mid < pulse_max')
        ctl = data['controller']
        if not rc['pulse_min'] <= ctl['switch_low'] < ctl['switch_high'] <= rc['pulse_max']:
            problems.append('controller: needs pulse_min <= switch_low < switch_high <= pulse_max')
    if problems:
        lines = '\n'.join(f'  - {p}' for p in problems)
        raise ElectronicsConfigError(f'Invalid electronics config {path}:\n{lines}')
    cfg = _namespace({k: data[k] for k in KEYS})
    cfg.name = data.get('name', os.path.splitext(os.path.basename(path))[0])
    cfg.path = path
    return cfg


def load_electronics_config(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise ElectronicsConfigError(f'{path}: file does not exist')
    with open(path, encoding='utf-8') as f:
        return parse_electronics_config(yaml.safe_load(f), path)


def config_dir():
    """Directory with the installed electronics configs (share/<pkg>/config)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('lawn_mower_electronics'), 'config')
    except (ImportError, LookupError):
        # Source tree, for tests without a colcon install
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def resolve_electronics_config(name):
    """Map a config name (faculty_robot) or a path to a config file path."""
    if os.sep in name or name.endswith(('.yaml', '.yml')):
        return os.path.abspath(os.path.expanduser(name))
    return os.path.join(config_dir(), f'{name}.yaml')
