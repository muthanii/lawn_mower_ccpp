import math
import xml.etree.ElementTree as ET

import pytest

from lawn_mower_electronics.sensors import (EARTH_RADIUS, GNSS_TOPIC, IMU_TOPIC, sensors_sdf,
                                            world_sdf)
from lawn_mower_robot.sdf_model import build_sdf


def test_sensors_are_mounted_on_the_chassis(electronics, robot):
    model = ET.fromstring(build_sdf(robot, sensors=sensors_sdf(electronics))).find('model')
    chassis = next(link for link in model.findall('link') if link.get('name') == 'chassis')
    sensors = {s.get('type'): s for s in chassis.findall('sensor')}
    assert sensors['imu'].find('topic').text == IMU_TOPIC
    assert sensors['navsat'].find('topic').text == GNSS_TOPIC
    assert float(sensors['navsat'].find('update_rate').text) == electronics.sensors.gnss.rate
    stddev = sensors['navsat'].find('navsat/position_sensing/horizontal/noise/stddev')
    # Gazebo takes the horizontal noise in degrees: 1 m is about 9e-6 deg.
    metres = math.radians(float(stddev.text)) * EARTH_RADIUS
    assert metres == pytest.approx(electronics.sensors.gnss.horizontal_stddev, rel=1e-4)


def test_bumper_watches_the_chassis_collision(electronics, robot):
    model = ET.fromstring(build_sdf(robot, sensors=sensors_sdf(electronics))).find('model')
    chassis = next(link for link in model.findall('link') if link.get('name') == 'chassis')
    bumper = next(s for s in chassis.findall('sensor') if s.get('type') == 'contact')
    watched = bumper.find('contact/collision').text
    assert watched in [c.get('name') for c in chassis.findall('collision')]


def test_model_without_sensors_is_unchanged(robot):
    model = ET.fromstring(build_sdf(robot)).find('model')
    assert not model.findall('link/sensor')


def test_world_has_the_sensor_systems_and_an_origin():
    world = ET.fromstring(f'<world>{world_sdf((32.5, 44.25))}</world>')
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert plugins == {'gz-sim-imu-system', 'gz-sim-navsat-system', 'gz-sim-contact-system'}
    assert float(world.find('spherical_coordinates/latitude_deg').text) == 32.5
    assert world.find('spherical_coordinates/world_frame_orientation').text == 'ENU'
