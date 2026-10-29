"""Load and validate a robot config file (config/robot/<name>.yaml).

Every node, the energy model and the Gazebo launch read the robot through
this module, so selecting a robot is only a matter of passing another file.
The loader fails loudly: it collects every problem in the file (missing or
unknown fields, TODO values, wrong units, unknown sources) and raises them
together in one RobotConfigError.
"""

import math
import os
from dataclasses import dataclass, field

import yaml

TODO = 'TODO'

# Units a value may be written in, and the factor to the schema unit.
UNIT_CONVERSIONS = {
    ('deg/s', 'rad/s'): math.pi / 180,
    ('deg/s^2', 'rad/s^2'): math.pi / 180,
}


@dataclass(frozen=True)
class Field:
    unit: str
    description: str
    # Used by a model, so it needs a number. Informational fields may be
    # null until a source is found, but they must still be listed.
    required: bool = True
    positive: bool = True


@dataclass(frozen=True)
class StringField:
    description: str
    required: bool = True


SCHEMA = {
    'geometry': {
        'track_width': Field('m', 'Lateral distance between left and right wheel centres (B).'),
        'wheelbase': Field('m', 'Longitudinal distance between front and rear axles (L).'),
        'wheel_radius': Field('m', 'Wheel radius (r).'),
        'wheel_width': Field('m', 'Tyre width.'),
        'body_length': Field('m', 'Overall length.'),
        'body_width': Field('m', 'Overall width.'),
        'body_height': Field('m', 'Overall height.'),
    },
    'inertia': {
        'mass': Field('kg', 'Total operating mass, batteries included (m).'),
        'wheel_mass': Field('kg', 'Mass of one wheel.'),
        'yaw_inertia': Field('kg*m^2', 'Moment of inertia about the vertical axis (I_z).'),
    },
    'drive': {
        'wheel_count': Field('1', 'Number of driven wheels.'),
        'gear_ratio': Field('1', 'Motor to wheel gear ratio.', required=False),
        'wheel_max_speed': Field('rad/s', 'Maximum wheel angular speed.'),
        'wheel_max_torque': Field('N*m', 'Maximum torque at one wheel.', required=False),
        'drivetrain_efficiency': Field('1', 'Battery to wheel efficiency (motor, gearbox, driver) (eta).'),
    },
    'motor': {
        'count': Field('1', 'Number of drive motors.'),
        'rated_voltage': Field('V', 'Rated motor voltage.'),
        'rated_current': Field('A', 'Rated motor current.', required=False),
        'rated_power': Field('W', 'Rated motor power.', required=False),
    },
    'battery': {
        'voltage': Field('V', 'Nominal battery voltage.'),
        'capacity': Field('A*h', 'Total battery capacity.'),
    },
    'limits': {
        'max_linear_speed': Field('m/s', 'Maximum forward speed (v_max).'),
        'max_angular_speed': Field('rad/s', 'Maximum yaw rate (omega_max).'),
        'max_linear_acceleration': Field('m/s^2', 'Maximum linear acceleration.'),
        'max_angular_acceleration': Field('rad/s^2', 'Maximum yaw acceleration.'),
    },
    'terrain': {
        'rolling_resistance': Field('1', 'Rolling resistance coefficient (mu_r).'),
        'lateral_friction': Field('1', 'Lateral sliding friction coefficient (mu_l).'),
        'longitudinal_friction': Field('1', 'Tyre-ground friction along the wheel, used by Gazebo.'),
        'icr_expansion_factor': Field('1', 'Skid-steer ICR expansion factor (chi >= 1).'),
        'longitudinal_slip': Field('1', 'Steady longitudinal slip ratio (s).', positive=False),
    },
    'electrical': {
        'auxiliary_power': Field('W', 'Constant draw of computer, sensors and drivers.', positive=False),
    },
    'gazebo': {
        'model_name': StringField('Name of the spawned model.'),
        'chassis_mesh': StringField('Chassis visual mesh URI, or null for a box.', required=False),
        'wheel_mesh': StringField('Wheel visual mesh URI, or null for a cylinder.', required=False),
        'chassis_mesh_offset_z': Field('m', 'Height of the chassis mesh origin above the wheel axles.',
                                       required=False, positive=False),
    },
}


class RobotConfigError(ValueError):
    """The robot config file is missing fields or has invalid values."""

    def __init__(self, path, problems):
        self.path = path
        self.problems = problems
        lines = '\n'.join(f'  - {p}' for p in problems)
        super().__init__(f'Invalid robot config {path} ({len(problems)} problems):\n{lines}')


class Group:
    """Values of one schema group, read as attributes (cfg.geometry.track_width)."""

    def __init__(self, name, values):
        self._name = name
        self._values = values

    def __getattr__(self, key):
        try:
            return self.__dict__['_values'][key]
        except KeyError:
            raise AttributeError(f'robot config has no field {self._name}.{key}') from None

    def as_dict(self):
        return dict(self._values)


@dataclass
class RobotConfig:
    name: str
    path: str
    groups: dict
    sources: dict
    # Fully qualified names of values that rest on a modelling assumption.
    assumptions: list = field(default_factory=list)

    def __getattr__(self, key):
        groups = self.__dict__.get('groups', {})
        if key in groups:
            return groups[key]
        raise AttributeError(key)

    def as_dict(self):
        return {name: group.as_dict() for name, group in self.groups.items()}


def _check_entry(qualified, spec, entry, sources, problems):
    """Validate one parameter entry and return its value in schema units."""
    if not isinstance(entry, dict) or 'value' not in entry:
        problems.append(f'{qualified}: expected a mapping with value, unit and source')
        return None

    value = entry['value']
    source = entry.get('source')
    if source in (None, '', TODO):
        problems.append(f'{qualified}: no source given')
    elif source not in sources:
        problems.append(f'{qualified}: source "{source}" is not listed under sources')

    if value == TODO:
        problems.append(f'{qualified}: value is TODO')
        return None
    if value is None:
        if spec.required:
            problems.append(f'{qualified}: value is null but the field is required')
        return None

    if isinstance(spec, StringField):
        if not isinstance(value, str):
            problems.append(f'{qualified}: expected a string, got {value!r}')
            return None
        return value

    unit = entry.get('unit')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        problems.append(f'{qualified}: expected a number, got {value!r}')
        return None
    if unit != spec.unit:
        factor = UNIT_CONVERSIONS.get((unit, spec.unit))
        if factor is None:
            problems.append(f'{qualified}: unit is "{unit}", expected "{spec.unit}"')
            return None
        value = value * factor
    if not math.isfinite(value) or value < 0 or (spec.positive and value == 0):
        problems.append(f'{qualified}: value {value!r} must be {"> 0" if spec.positive else ">= 0"}')
        return None
    return float(value)


def parse_robot_config(data, path='<memory>'):
    """Validate a parsed YAML document and return a RobotConfig."""
    problems = []
    if not isinstance(data, dict):
        raise RobotConfigError(path, ['file is not a YAML mapping'])

    name = data.get('name')
    if not isinstance(name, str) or not name or name == TODO:
        problems.append('name: missing')
    sources = data.get('sources')
    if not isinstance(sources, dict) or not sources:
        problems.append('sources: missing; list every reference the values cite')
        sources = {}

    params = data.get('parameters')
    if not isinstance(params, dict):
        raise RobotConfigError(path, problems + ['parameters: missing'])

    for group in sorted(set(params) - set(SCHEMA)):
        problems.append(f'parameters.{group}: unknown group')

    groups = {}
    assumptions = []
    for group, fields in SCHEMA.items():
        section = params.get(group)
        if not isinstance(section, dict):
            problems.append(f'{group}: missing group')
            continue
        for key in sorted(set(section) - set(fields)):
            problems.append(f'{group}.{key}: unknown field')
        values = {}
        for key, spec in fields.items():
            qualified = f'{group}.{key}'
            if key not in section:
                problems.append(f'{qualified}: missing field ({spec.description})')
                continue
            entry = section[key]
            values[key] = _check_entry(qualified, spec, entry, sources, problems)
            if isinstance(entry, dict) and entry.get('source') == 'assumption':
                assumptions.append(qualified)
        groups[group] = Group(group, values)

    if 'terrain' in groups and (groups['terrain'].as_dict().get('icr_expansion_factor') or 1) < 1:
        problems.append('terrain.icr_expansion_factor: must be >= 1')

    if problems:
        raise RobotConfigError(path, problems)
    return RobotConfig(name=name, path=path, groups=groups, sources=sources,
                       assumptions=assumptions)


def load_robot_config(path):
    """Read and validate a robot config file. Raises RobotConfigError."""
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise RobotConfigError(path, ['file does not exist'])
    with open(path, encoding='utf-8') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise RobotConfigError(path, [f'YAML syntax error: {e}']) from None
    return parse_robot_config(data, path)


def config_dir():
    """Directory with the installed robot configs (share/<pkg>/config/robot)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('lawn_mower_robot'), 'config', 'robot')
    except (ImportError, LookupError):
        # Source tree, for tests without a colcon install
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'config', 'robot')


def resolve_robot_config(robot):
    """Map a robot name (pioneer3at) or a path to a config file path."""
    if os.sep in robot or robot.endswith(('.yaml', '.yml')):
        return os.path.abspath(os.path.expanduser(robot))
    return os.path.join(config_dir(), f'{robot}.yaml')
