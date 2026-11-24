import xml.etree.ElementTree as ET

from lawn_mower_electronics.sensors import IMU_TOPIC, sensors_sdf, world_sdf
from lawn_mower_robot.sdf_model import build_sdf


def test_sensors_are_mounted_on_the_chassis(electronics, robot):
    model = ET.fromstring(build_sdf(robot, sensors=sensors_sdf(electronics))).find('model')
    chassis = next(link for link in model.findall('link') if link.get('name') == 'chassis')
    sensors = {s.get('type'): s for s in chassis.findall('sensor')}
    assert set(sensors) == {'imu', 'contact'}
    assert sensors['imu'].find('topic').text == IMU_TOPIC
    assert float(sensors['imu'].find('update_rate').text) == electronics.sensors.imu.rate


def test_bumper_watches_the_chassis_collision(electronics, robot):
    model = ET.fromstring(build_sdf(robot, sensors=sensors_sdf(electronics))).find('model')
    chassis = next(link for link in model.findall('link') if link.get('name') == 'chassis')
    bumper = next(s for s in chassis.findall('sensor') if s.get('type') == 'contact')
    watched = bumper.find('contact/collision').text
    assert watched in [c.get('name') for c in chassis.findall('collision')]


def test_model_without_sensors_is_unchanged(robot):
    model = ET.fromstring(build_sdf(robot)).find('model')
    assert not model.findall('link/sensor')


def test_world_has_the_sensor_systems():
    world = ET.fromstring(f'<world>{world_sdf()}</world>')
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert plugins == {'gz-sim-imu-system', 'gz-sim-contact-system'}
