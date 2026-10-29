"""Build the Gazebo SDF model of the robot from its config.

The physics (geometry, masses, inertia, friction, drive limits) comes only
from the config, so the simulated robot always matches the one the planner and
energy model use. The downloaded Gazebo model only gives the visual meshes
(gazebo.chassis_mesh and gazebo.wheel_mesh); without them the robot is drawn
as a box and four cylinders.

The skid-steer drive is Gazebo's DiffDrive system with two joints per side and
the effective track width chi * B, so a cmd_vel (v, omega) gives the same wheel
speeds as the kinematic model.
"""

import math
from xml.sax.saxutils import escape

from .kinematics import SkidSteerKinematics

# Gazebo physics has no continuous joint, so wheels are revolute without limits.
UNLIMITED = 1e16

WHEELS = (('front_left', 1, 1), ('rear_left', -1, 1),
          ('front_right', 1, -1), ('rear_right', -1, -1))


def _box_inertia(mass, x, y, z):
    return (mass * (y ** 2 + z ** 2) / 12, mass * (x ** 2 + z ** 2) / 12,
            mass * (x ** 2 + y ** 2) / 12)


def _cylinder_inertia(mass, radius, length):
    """Inertia of a cylinder with its axis along the link's z axis."""
    side = mass * (3 * radius ** 2 + length ** 2) / 12
    return side, side, mass * radius ** 2 / 2


def _inertial(mass, ixx, iyy, izz):
    return (f'<inertial><mass>{mass:.6g}</mass><inertia>'
            f'<ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def wheel_positions(cfg):
    """(name, x, y) of each wheel centre in the base frame."""
    half_l = cfg.geometry.wheelbase / 2
    half_b = cfg.geometry.track_width / 2
    return [(name, sx * half_l, sy * half_b) for name, sx, sy in WHEELS]


def chassis_inertia(cfg):
    """Chassis mass and diagonal inertia, so the whole robot has the config's
    mass and yaw inertia I_z."""
    g = cfg.geometry
    wheel_mass = cfg.inertia.wheel_mass
    wheels = wheel_positions(cfg)
    chassis_mass = cfg.inertia.mass - len(wheels) * wheel_mass
    if chassis_mass <= 0:
        raise ValueError('inertia.mass must be larger than the wheel masses')

    # Wheel axes point along y, so their own yaw inertia is the cylinder's side term.
    wheel_side, _, _ = _cylinder_inertia(wheel_mass, g.wheel_radius, g.wheel_width)
    wheels_yaw = sum(wheel_side + wheel_mass * (x ** 2 + y ** 2) for _, x, y in wheels)
    ixx, iyy, box_izz = _box_inertia(chassis_mass, g.body_length,
                                     g.track_width - g.wheel_width, chassis_height(cfg))
    izz = cfg.inertia.yaw_inertia - wheels_yaw
    if izz <= 0:
        # The config's I_z is below what the wheels alone give; fall back to the box.
        izz = box_izz
    return chassis_mass, ixx, iyy, izz


def chassis_height(cfg):
    """The chassis box spans from the wheel axles to the top of the body."""
    return cfg.geometry.body_height - cfg.geometry.wheel_radius


def _visual(name, geometry_xml, pose='0 0 0 0 0 0'):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry>{geometry_xml}</geometry></visual>'


def _mesh(uri):
    return f'<mesh><uri>{escape(uri)}</uri></mesh>'


def build_sdf(cfg):
    """Return the SDF model of the robot as a string."""
    g = cfg.geometry
    t = cfg.terrain
    gz = cfg.gazebo
    lim = cfg.limits
    kin = SkidSteerKinematics.from_config(cfg)
    name = gz.model_name

    body_h = chassis_height(cfg)
    body_w = g.track_width - g.wheel_width
    chassis_mass, ixx, iyy, izz = chassis_inertia(cfg)
    if gz.chassis_mesh:
        offset = gz.chassis_mesh_offset_z or 0.0
        chassis_visual = _visual('visual', _mesh(gz.chassis_mesh), f'0 0 {offset:.6g} 0 0 0')
    else:
        chassis_visual = _visual(
            'visual', f'<box><size>{g.body_length:.6g} {body_w:.6g} {body_h:.6g}</size></box>',
            f'0 0 {body_h / 2:.6g} 0 0 0')

    links = [f"""
    <link name="chassis">
      <pose>0 0 {g.wheel_radius:.6g} 0 0 0</pose>
      {_inertial(chassis_mass, ixx, iyy, izz)}
      <collision name="collision">
        <pose>0 0 {body_h / 2:.6g} 0 0 0</pose>
        <geometry><box><size>{g.body_length:.6g} {body_w:.6g} {body_h:.6g}</size></box></geometry>
      </collision>
      {chassis_visual}
    </link>"""]

    wheel_inertia = _cylinder_inertia(cfg.inertia.wheel_mass, g.wheel_radius, g.wheel_width)
    cylinder = (f'<cylinder><radius>{g.wheel_radius:.6g}</radius>'
                f'<length>{g.wheel_width:.6g}</length></cylinder>')
    wheel_visual = _mesh(gz.wheel_mesh) if gz.wheel_mesh else cylinder
    joints = []
    for wheel, x, y in wheel_positions(cfg):
        # Roll by pi/2 so the cylinder axis (link z) points along the base y axis.
        links.append(f"""
    <link name="{wheel}_wheel">
      <pose>{x:.6g} {y:.6g} {g.wheel_radius:.6g} {math.pi / 2:.6g} 0 0</pose>
      {_inertial(cfg.inertia.wheel_mass, *wheel_inertia)}
      <collision name="collision">
        <geometry>{cylinder}</geometry>
        <!-- fdir1 is the wheel axle (link z): mu acts sideways, mu2 along the rolling direction -->
        <surface><friction><ode>
          <mu>{t.lateral_friction:.6g}</mu>
          <mu2>{t.longitudinal_friction:.6g}</mu2>
          <fdir1>0 0 1</fdir1>
        </ode></friction></surface>
      </collision>
      {_visual('visual', wheel_visual)}
    </link>""")
        joints.append(f"""
    <joint name="{wheel}_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>{wheel}_wheel</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>{-UNLIMITED:g}</lower><upper>{UNLIMITED:g}</upper></limit></axis>
    </joint>""")

    def joint_tags(tag, side):
        return '\n      '.join(f'<{tag}>{w}_wheel_joint</{tag}>'
                               for w, _, _ in WHEELS if w.endswith(side))

    plugins = f"""
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      {joint_tags('left_joint', 'left')}
      {joint_tags('right_joint', 'right')}
      <wheel_separation>{kin.effective_track_width:.6g}</wheel_separation>
      <wheel_radius>{kin.effective_wheel_radius:.6g}</wheel_radius>
      <max_linear_velocity>{lim.max_linear_speed:.6g}</max_linear_velocity>
      <min_linear_velocity>{-lim.max_linear_speed:.6g}</min_linear_velocity>
      <max_angular_velocity>{lim.max_angular_speed:.6g}</max_angular_velocity>
      <min_angular_velocity>{-lim.max_angular_speed:.6g}</min_angular_velocity>
      <max_linear_acceleration>{lim.max_linear_acceleration:.6g}</max_linear_acceleration>
      <min_linear_acceleration>{-lim.max_linear_acceleration:.6g}</min_linear_acceleration>
      <max_angular_acceleration>{lim.max_angular_acceleration:.6g}</max_angular_acceleration>
      <min_angular_acceleration>{-lim.max_angular_acceleration:.6g}</min_angular_acceleration>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{escape(name)}</robot_base_frame>
      <odom_topic>/ground_truth/odom</odom_topic>
      <dimensions>2</dimensions>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>/joint_states</topic>
    </plugin>"""

    return (f'<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="{escape(name)}">'
            + ''.join(links) + ''.join(joints) + plugins + '\n  </model>\n</sdf>\n')


def main(argv=None):
    """Print the SDF for a robot config: robot_sdf pioneer3at [-o model.sdf]."""
    import argparse

    from .robot_config import load_robot_config, resolve_robot_config

    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('robot', help='config name (pioneer3at) or path to a YAML file')
    parser.add_argument('-o', '--output', help='write to this file instead of stdout')
    args = parser.parse_args(argv)
    sdf = build_sdf(load_robot_config(resolve_robot_config(args.robot)))
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(sdf)
    else:
        print(sdf, end='')


if __name__ == '__main__':
    main()
