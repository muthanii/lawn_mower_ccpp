"""Topic names of the navigation nodes. The electronics topics are in
lawn_mower_electronics.topics."""

POSE = '/localization/odom'        # Odometry, the estimated pose in the lawn frame
MISSION_PATH = '/mission/path'     # Path, the planned coverage path (latched)
MISSION_STATUS = '/mission/status'  # String: PLANNED, WAITING, RUNNING, PAUSED, LOST or DONE
WHEEL_ODOM = '/odom'               # Odometry from wheel encoders, where a robot has them
FRAME = 'map'                      # x east, y north, origin at the lawn's (0, 0)
BASE_FRAME = 'base_link'
