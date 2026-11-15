"""Topic names of the electronics signal chain, in the order of the wiring.

    transmitter -> receiver -> controller -> motor driver -> Gazebo drive
                                   |      -> blade relay  -> blade motor
       companion computer ---------+
"""

RC_TRANSMITTER = '/rc/transmitter'   # UInt16MultiArray, radio frames [us]; silence = no link
RC_CHANNELS = '/rc/channels'         # UInt16MultiArray, receiver outputs [us]
RC_FAILSAFE = '/rc/failsafe'         # Bool, the receiver lost the transmitter
CMD_VEL = '/cmd_vel'                 # Twist, companion -> controller
BLADE_REQUEST = '/blade/request'     # Bool, companion -> controller
ESTOP = '/estop'                     # Bool, emergency stop button pressed
CONTROLLER_STATE = '/controller/state'  # String: MANUAL, AUTO, FAILSAFE or ESTOP
PWM_DRIVE = '/pwm/drive'             # UInt16MultiArray [left, right], controller -> driver [us]
BLADE_RELAY = '/blade/relay'         # Bool, controller -> blade relay
MOTOR_CMD_VEL = '/motor/cmd_vel'     # Twist, driver -> Gazebo DiffDrive
WHEEL_SPEEDS = '/motor/wheel_speeds'  # Float64MultiArray [left, right] [rad/s]
BLADE_SPEED = '/blade/speed'         # Float64 [rpm]
BLADE_RUNNING = '/blade/running'     # Bool, the blade is fast enough to cut
BATTERY = '/battery'                 # BatteryState
ENERGY = '/energy'                   # Float64MultiArray [traction, blade, auxiliary] [J]
GROUND_TRUTH = '/ground_truth/odom'  # Odometry from Gazebo, the measured motion
