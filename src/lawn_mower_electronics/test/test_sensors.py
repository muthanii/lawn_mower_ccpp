import xml.etree.ElementTree as ET

from lawn_mower_electronics.sensors import GNSS_TOPIC, IMU_TOPIC, sensors_sdf, world_sdf
from lawn_mower_robot.sdf_model import build_sdf


def test_sensors_are_mounted_on_the_chassis(electronics, robot):
    model = ET.fromstring(build_sdf(robot, sensors=sensors_sdf(electronics))).find('model')
    chassis = next(link for link in model.findall('link') if link.get('name') == 'chassis')
    sensors = {s.get('type'): s for s in chassis.findall('sensor')}
    assert sensors['imu'].find('topic').text == IMU_TOPIC
    assert sensors['navsat'].find('topic').text == GNSS_TOPIC
    assert float(sensors['navsat'].find('update_rate').text) == electronics.sensors.gnss.rate
    stddev = sensors['navsat'].find('navsat/position_sensing/horizontal/noise/stddev')
    assert float(stddev.text) == electronics.sensors.gnss.horizontal_stddev


def test_model_without_sensors_is_unchanged(robot):
    model = ET.fromstring(build_sdf(robot)).find('model')
    assert not model.findall('link/sensor')


def test_world_has_the_sensor_systems_and_an_origin():
    world = ET.fromstring(f'<world>{world_sdf(32.5, 44.25)}</world>')
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert plugins == {'gz-sim-imu-system', 'gz-sim-navsat-system'}
    assert float(world.find('spherical_coordinates/latitude_deg').text) == 32.5
    assert world.find('spherical_coordinates/world_frame_orientation').text == 'ENU'
