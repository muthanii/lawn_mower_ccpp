import xml.etree.ElementTree as ET

import pytest

from lawn_mower_robot.kinematics import SkidSteerKinematics
from lawn_mower_robot.sdf_model import build_sdf, chassis_inertia, wheel_positions


@pytest.fixture
def sdf(pioneer):
    return ET.fromstring(build_sdf(pioneer))


def test_drive_plugin_uses_config(sdf, pioneer):
    kin = SkidSteerKinematics.from_config(pioneer)
    plugin = sdf.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(plugin.find('wheel_separation').text) == pytest.approx(kin.effective_track_width, rel=1e-5)
    assert float(plugin.find('wheel_radius').text) == pytest.approx(kin.effective_wheel_radius, rel=1e-5)
    assert float(plugin.find('max_linear_velocity').text) == pytest.approx(
        pioneer.limits.max_linear_speed)
    assert len(plugin.findall('left_joint')) == 2
    assert len(plugin.findall('right_joint')) == 2


def test_total_mass_matches_config(sdf, pioneer):
    total = sum(float(m.text) for m in sdf.iter('mass'))
    assert total == pytest.approx(pioneer.inertia.mass, rel=1e-5)


def test_total_yaw_inertia_matches_config(pioneer):
    _, _, _, izz = chassis_inertia(pioneer)
    g, wm = pioneer.geometry, pioneer.inertia.wheel_mass
    wheel_side = wm * (3 * g.wheel_radius ** 2 + g.wheel_width ** 2) / 12
    wheels = sum(wheel_side + wm * (x ** 2 + y ** 2) for _, x, y in wheel_positions(pioneer))
    assert izz + wheels == pytest.approx(pioneer.inertia.yaw_inertia)


def test_wheels_sit_on_the_ground(sdf, pioneer):
    for link in sdf.iter('link'):
        if link.get('name').endswith('_wheel'):
            z = float(link.find('pose').text.split()[2])
            assert z == pytest.approx(pioneer.geometry.wheel_radius)


def test_meshes_come_from_the_downloaded_model(sdf):
    uris = {u.text for u in sdf.iter('uri')}
    assert uris == {'model://pioneer3at/meshes/chassis.dae', 'model://pioneer3at/meshes/wheel.dae'}


def test_wheel_joints_are_revolute(sdf):
    # Gazebo turns continuous joints into fixed ones.
    joints = sdf.findall('.//joint')
    assert len(joints) == 4
    assert all(j.get('type') == 'revolute' for j in joints)
