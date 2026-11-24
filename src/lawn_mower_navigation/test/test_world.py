import xml.etree.ElementTree as ET

from lawn_mower_navigation.lawns import DEMO, find_lawn
from lawn_mower_navigation.world import lawn_world_sdf


def world_of(lawn, **kwargs):
    return ET.fromstring(lawn_world_sdf(lawn.area, **kwargs)).find('world')


def test_one_obstacle_model_per_hole():
    for lawn in (DEMO, find_lawn('backyard')):
        names = [m.get('name') for m in world_of(lawn).findall('model')]
        obstacles = [n for n in names if n.startswith('obstacle_')]
        assert len(obstacles) == len(lawn.area.interiors)
        assert {'ground_plane', 'lawn_boundary'} <= set(names)


def test_obstacle_has_the_corners_of_the_hole():
    world = world_of(DEMO)
    obstacle = next(m for m in world.findall('model') if m.get('name') == 'obstacle_0')
    points = {tuple(float(v) for v in p.text.split())
              for p in obstacle.findall('link/collision/geometry/polyline/point')}
    assert points == {(5.0, 3.5), (7.0, 3.5), (7.0, 5.5), (5.0, 5.5)}


def test_boundary_has_no_collision():
    world = world_of(DEMO)
    boundary = next(m for m in world.findall('model') if m.get('name') == 'lawn_boundary')
    assert not boundary.findall('link/collision')
    assert len(boundary.findall('link/visual')) == 4


def test_world_has_the_sensor_systems():
    world = world_of(DEMO)
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert {'gz-sim-physics-system', 'gz-sim-imu-system', 'gz-sim-contact-system'} <= plugins
    assert 'gz-sim-navsat-system' not in plugins


def test_step_and_speed_are_arguments():
    world = world_of(DEMO, step=0.002, real_time_factor=1.0)
    assert float(world.find('physics/max_step_size').text) == 0.002
    assert float(world.find('physics/real_time_factor').text) == 1.0
