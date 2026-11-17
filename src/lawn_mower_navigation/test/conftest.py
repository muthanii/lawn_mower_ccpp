import pytest

from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config


@pytest.fixture(scope='session')
def robot():
    return load_robot_config(resolve_robot_config('faculty_robot_estimate'))
