"""SDF of the sensors that the retrofit adds, for Gazebo.

sensors_sdf gives the <sensor> elements to mount on the chassis (pass it to
lawn_mower_robot.sdf_model.build_sdf). world_sdf gives the world plugins and
the geographic origin that the sensors need.
"""

import math

IMU_TOPIC = '/imu'
GNSS_TOPIC = '/navsat'
CONTACT_TOPIC = '/bumper/contacts'
CHASSIS_COLLISION = 'collision'   # name of the chassis collision in the robot model
EARTH_RADIUS = 6371000.0  # m, mean


def metres_to_degrees(distance):
    """Angle [deg] that a distance [m] spans on the Earth's surface.

    Gazebo adds the horizontal GNSS noise to the latitude and longitude in
    degrees. The east error in metres is this times cos(latitude).
    """
    return math.degrees(distance / EARTH_RADIUS)


def _noise(stddev):
    return f'<noise type="gaussian"><mean>0</mean><stddev>{stddev:.6g}</stddev></noise>'


def sensors_sdf(electronics):
    """IMU, GNSS receiver and bumper on the chassis link.

    The bumper is a contact sensor on the chassis collision. The chassis does
    not touch the ground, so every contact is an obstacle.
    """
    gnss, imu = electronics.sensors.gnss, electronics.sensors.imu
    gyro = ''.join(f'<{a}>{_noise(imu.gyro_stddev)}</{a}>' for a in 'xyz')
    accel = ''.join(f'<{a}>{_noise(imu.accel_stddev)}</{a}>' for a in 'xyz')
    return f"""
      <sensor name="imu" type="imu">
        <always_on>1</always_on>
        <update_rate>{imu.rate:.6g}</update_rate>
        <topic>{IMU_TOPIC}</topic>
        <imu>
          <angular_velocity>{gyro}</angular_velocity>
          <linear_acceleration>{accel}</linear_acceleration>
        </imu>
      </sensor>
      <sensor name="gnss" type="navsat">
        <always_on>1</always_on>
        <update_rate>{gnss.rate:.6g}</update_rate>
        <topic>{GNSS_TOPIC}</topic>
        <navsat>
          <position_sensing>
            <horizontal>{_noise(metres_to_degrees(gnss.horizontal_stddev))}</horizontal>
            <vertical>{_noise(gnss.vertical_stddev)}</vertical>
          </position_sensing>
        </navsat>
      </sensor>
      <sensor name="bumper" type="contact">
        <always_on>1</always_on>
        <update_rate>{electronics.controller.rate:.6g}</update_rate>
        <contact>
          <collision>{CHASSIS_COLLISION}</collision>
          <topic>{CONTACT_TOPIC}</topic>
        </contact>
      </sensor>"""


def world_sdf(origin, elevation=0.0):
    """World plugins for the sensors, and the geographic origin of the world.

    origin is (degrees north, degrees east) of the world's (0, 0); x points
    east and y north.
    """
    north, east = origin
    return f"""
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{north:.9f}</latitude_deg>
      <longitude_deg>{east:.9f}</longitude_deg>
      <elevation>{elevation:.6g}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>"""
