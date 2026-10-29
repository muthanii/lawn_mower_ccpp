import copy
import math
import os

import pytest
import yaml

from lawn_mower_robot.robot_config import (SCHEMA, RobotConfigError, load_robot_config,
                                           parse_robot_config, resolve_robot_config)

from conftest import CONFIG_DIR


def raw(name='pioneer3at'):
    with open(os.path.join(CONFIG_DIR, f'{name}.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


def problems(data):
    with pytest.raises(RobotConfigError) as e:
        parse_robot_config(data)
    return e.value.problems


def test_pioneer_loads_every_schema_field(pioneer):
    for group, fields in SCHEMA.items():
        for key in fields:
            getattr(getattr(pioneer, group), key)
    assert pioneer.name == 'pioneer3at'


def test_degrees_are_converted(pioneer):
    assert pioneer.limits.max_angular_speed == pytest.approx(math.radians(140))


def test_assumptions_are_reported(pioneer):
    assert 'drive.drivetrain_efficiency' in pioneer.assumptions
    assert 'geometry.track_width' not in pioneer.assumptions


def test_every_value_cites_a_listed_source():
    data = raw()
    for group in data['parameters'].values():
        for entry in group.values():
            assert entry['source'] in data['sources']


def test_missing_field_fails_loudly():
    data = raw()
    del data['parameters']['geometry']['track_width']
    assert any('geometry.track_width: missing' in p for p in problems(data))


def test_missing_group_fails():
    data = raw()
    del data['parameters']['terrain']
    assert 'terrain: missing group' in problems(data)


def test_unknown_field_is_a_typo():
    data = raw()
    data['parameters']['geometry']['track_widht'] = data['parameters']['geometry']['track_width']
    assert 'geometry.track_widht: unknown field' in problems(data)


def test_wrong_unit_fails():
    data = raw()
    data['parameters']['geometry']['wheel_radius']['unit'] = 'mm'
    assert any('unit is "mm"' in p for p in problems(data))


def test_unknown_source_fails():
    data = raw()
    data['parameters']['inertia']['mass']['source'] = 'nowhere'
    assert any('inertia.mass: source "nowhere"' in p for p in problems(data))


def test_required_null_fails_but_informational_null_passes():
    data = raw()
    assert data['parameters']['drive']['gear_ratio']['value'] is None
    parse_robot_config(copy.deepcopy(data))
    data['parameters']['inertia']['mass']['value'] = None
    assert any('inertia.mass: value is null' in p for p in problems(data))


def test_non_positive_value_fails():
    data = raw()
    data['parameters']['inertia']['mass']['value'] = 0
    assert any('inertia.mass: value 0' in p for p in problems(data))


def test_expansion_factor_below_one_fails():
    data = raw()
    data['parameters']['terrain']['icr_expansion_factor']['value'] = 0.8
    assert any('icr_expansion_factor: must be >= 1' in p for p in problems(data))


def test_all_problems_reported_together():
    data = raw()
    del data['parameters']['geometry']['track_width']
    data['parameters']['inertia']['mass']['value'] = 'TODO'
    assert len(problems(data)) == 2


def test_faculty_stub_fails_and_lists_todos():
    with pytest.raises(RobotConfigError) as e:
        load_robot_config(os.path.join(CONFIG_DIR, 'faculty_robot.yaml'))
    message = str(e.value)
    assert 'geometry.track_width: value is TODO' in message
    assert 'terrain.icr_expansion_factor: value is TODO' in message


def test_faculty_stub_has_the_same_fields_as_pioneer():
    pioneer, faculty = raw(), raw('faculty_robot')
    assert {g: set(f) for g, f in pioneer['parameters'].items()} == \
        {g: set(f) for g, f in faculty['parameters'].items()}


def test_missing_file_fails():
    with pytest.raises(RobotConfigError, match='does not exist'):
        load_robot_config('/nonexistent/robot.yaml')


def test_resolve_name_and_path():
    assert resolve_robot_config('pioneer3at').endswith(os.path.join('robot', 'pioneer3at.yaml'))
    assert resolve_robot_config('/tmp/x.yaml') == '/tmp/x.yaml'
