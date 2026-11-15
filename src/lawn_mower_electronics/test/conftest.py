import os

import pytest

from lawn_mower_electronics.config import load_electronics_config
from lawn_mower_robot.kinematics import SkidSteerKinematics
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


@pytest.fixture
def electronics():
    return load_electronics_config(os.path.join(CONFIG_DIR, 'faculty_robot.yaml'))


@pytest.fixture
def robot():
    return load_robot_config(resolve_robot_config('faculty_robot_estimate'))


@pytest.fixture
def kinematics(robot):
    return SkidSteerKinematics.from_config(robot)


def frame(electronics, **named):
    """A transmitter frame with every channel centred, except the named ones."""
    rc = electronics.rc
    out = [rc.pulse_mid] * rc.channel_count
    for name, pulse in named.items():
        out[getattr(rc.channels, name) - 1] = pulse
    return out
