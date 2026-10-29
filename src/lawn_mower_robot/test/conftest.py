import os

import pytest

from lawn_mower_robot.robot_config import load_robot_config

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config', 'robot')


@pytest.fixture
def pioneer():
    return load_robot_config(os.path.join(CONFIG_DIR, 'pioneer3at.yaml'))
