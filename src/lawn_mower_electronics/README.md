# lawn_mower_electronics

Simulation models of the robot's electronics: the parts that are on the robot today, and the parts that the autonomy retrofit adds.

The models are behavioural. They reproduce what each part does to its signals (pulse widths, switch states, lags, timeouts), not its firmware. A node of this package can be replaced by the real part, because the topics follow the wiring.

## Signal chain

```text
transmitter ──radio──> receiver ──> controller ──pulses──> motor driver ──> wheels (Gazebo)
                                        │   └────relay───> blade relay ───> blade motor
       companion computer (ROS 2) ──────┘
       velocity command, blade request        emergency stop ──> controller, driver and blade supply
```

| Part | On the robot | Model | Node |
| --- | --- | --- | --- |
| Transmitter | Microzone 6C-MINI | A scripted operator, driven by parameters. | `rc_transmitter` |
| Receiver | Microzone MC7RB-V2 | Repeats the last frame. Outputs its failsafe values when the link is lost. | `rc_receiver` |
| Controller | Added by the retrofit | Manual and automatic modes, failsafe, emergency stop, command watchdog. | `controller` |
| Motor driver | L-faster dual channel | Deadband, linear output, first-order lag, signal timeout. | `motor_driver` |
| Blade | Relay chain and 24 V motor | One switch, with a spin-up lag and a power draw. | `blade` |
| Battery monitor | Added by the retrofit | Traction, blade and auxiliary energy, voltage sag, state of charge. | `power_monitor` |
| GNSS receiver and IMU | Added by the retrofit | Gazebo sensors with noise (`sensors.py`). | Gazebo |

The emergency stop is a hardware cut on the real robot. In the model it removes the supply of the motor driver and of the blade, and the controller latches it.

## Controller states

| State | When | Drive | Blade |
| --- | --- | --- | --- |
| `ESTOP` | The emergency stop is pressed, or was pressed and is not re-armed. | Neutral | Off |
| `FAILSAFE` | The receiver lost the transmitter. | Neutral | Off |
| `MANUAL` | The mode switch (CH5) is low. | The sticks, as before the retrofit | The blade switch (CH6) |
| `AUTO` | The mode switch is high. | The velocity command | On when the companion asks for it and the blade switch allows it |

- To re-arm after an emergency stop, release the stop and put the mode switch to manual.
- In `AUTO`, a velocity command older than `command_timeout` stops the drive and the blade.
- The robot does not drive itself without a radio link. A lost link stops it in every mode.

## Config

`config/faculty_robot.yaml` holds the values of every model. The radio values come from the Microzone 6C-MINI manual and the channel wiring from the faculty's circuit diagram. The values of the added parts are typical figures. Replace them when the parts are chosen and tested.

`load_electronics_config()` fails with a list of every missing or unknown key.

Set the receiver failsafe of the real radio to the values in `rc.failsafe`: CH5 and CH6 low. With the factory values, a lost link leaves CH6 at the centre.

## Run

All nodes except the transmitter run in one process:

```bash
ros2 run lawn_mower_electronics electronics --ros-args -p robot:=faculty_robot_estimate -p electronics:=faculty_robot
ros2 run lawn_mower_electronics rc_transmitter
```

Play the operator with parameters:

```bash
ros2 param set /rc_transmitter mode auto       # mode switch, CH5
ros2 param set /rc_transmitter blade true      # blade switch, CH6
ros2 param set /rc_transmitter throttle 0.5    # sticks, -1 to 1
ros2 param set /rc_transmitter steer -0.3      # positive turns left
ros2 param set /rc_transmitter link false      # transmitter off or out of range
ros2 topic pub /estop std_msgs/msg/Bool "data: true"
```

The topic names are in `topics.py`. `lawn_mower_navigation` starts these nodes together with Gazebo.

## From the model to the robot

| Model | Real part | What to check |
| --- | --- | --- |
| `controller` | A real-time board that reads the receiver's SBUS output and writes two servo pulses and one relay output. | The same state table. The tests in `test/test_controller.py` are its specification. |
| `motor_driver` | The L-faster driver. | Its deadband and its pulse range. Put them in the config. |
| `rc_receiver` | The MC7RB-V2. | The failsafe values, with the transmitter off. |
| `power_monitor` | A voltage and current sensor on the battery. | The energy model against measured energy. |
| GNSS and IMU | An RTK receiver and an IMU. | The noise values in the config. |
