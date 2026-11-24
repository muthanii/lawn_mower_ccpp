"""SDF of the sensors that the retrofit adds, for Gazebo.

sensors_sdf gives the <sensor> elements to mount on the chassis (pass it to
lawn_mower_robot.sdf_model.build_sdf). world_sdf gives the world plugins that
the sensors need.

There is no GNSS receiver for now: the mission takes the robot's pose from
the simulator (lawn_mower_navigation's sim_pose node).
"""

IMU_TOPIC = '/imu'
CONTACT_TOPIC = '/bumper/contacts'
CHASSIS_COLLISION = 'collision'   # name of the chassis collision in the robot model


def _noise(stddev):
    return f'<noise type="gaussian"><mean>0</mean><stddev>{stddev:.6g}</stddev></noise>'


def sensors_sdf(electronics):
    """IMU and bumper on the chassis link.

    The bumper is a contact sensor on the chassis collision. The chassis does
    not touch the ground, so every contact is an obstacle.
    """
    imu = electronics.sensors.imu
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
      <sensor name="bumper" type="contact">
        <always_on>1</always_on>
        <update_rate>{electronics.controller.rate:.6g}</update_rate>
        <contact>
          <collision>{CHASSIS_COLLISION}</collision>
          <topic>{CONTACT_TOPIC}</topic>
        </contact>
      </sensor>"""


def world_sdf():
    """World plugins for the sensors."""
    return """
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>"""
