# lawn_mower_navigation

Autonomous coverage for the lawn mower robot, and its simulation in Gazebo: localisation, a fuzzy path tracker, the mission, and an evaluator that scores the run.

## Run the simulation

```bash
ros2 launch lawn_mower_navigation coverage_sim.launch.py
```

The launch starts Gazebo with the lawn and the robot, the electronics models of `lawn_mower_electronics`, the localisation, the mission and the evaluator. A scripted operator switches the radio to automatic mode after 3 s. The robot then mows the lawn, and the launch ends when the results are written.

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot` | `faculty_robot_estimate` | Robot config. |
| `electronics` | `faculty_robot` | Electronics config. |
| `lawn` | `demo` | `demo`, or a lawn of `lawn_mower_planning.maps`, for example `two_beds`. |
| `method` | `energy_ga` | Cell ordering method of `lawn_mower_planning`. |
| `seed`, `budget` | `0`, `0` | Seed and evaluation budget of the method. `0` is the method's default budget. `energy_direction_ga` needs more than its direction choice uses. |
| `speed_fraction` | `0.5` | Driving speed as a fraction of the robot's limits. |
| `clearance` | `-1` | Distance the robot centre keeps from edges and obstacles [m]. `-1` is the radius the body sweeps in a turn, plus 0.1 m. |
| `speed_source` | `pulses` | Speed for the localisation: `pulses` (no encoders) or `odometry`. |
| `real_time_factor` | `1` | Simulation speed. The runs below used 2 and 3. Without a limit (`0`, about 11 on a 4-core machine) the message delays, which stay the same in real time, disturbed the control. |
| `headless` | `false` | Run without the Gazebo window. |
| `results_dir` | `mission_results` | Folder for the results. |

The results are in `results_dir`:

- `mission.csv`: one row for each run, with the duration, distance, coverage, overlap, tracking error, localisation error and the traction, blade and auxiliary energy.
- `trajectory.csv`: the true path of the last run, with the blade state.

## Nodes

| Node | Input | Output |
| --- | --- | --- |
| `localization` | GNSS fixes, gyro, pulses to the motor driver | `/localization/odom` |
| `mission` | Pose, controller state, blade state | `/cmd_vel`, `/blade/request`, `/mission/path`, `/mission/status` |
| `evaluator` | Gazebo ground truth, blade state, energy | `mission.csv`, `trajectory.csv` |

### Localisation

`localization.py` is an extended Kalman filter on the pose. The gyro and the forward speed drive the prediction, and each GNSS fix corrects the position. The robot has no wheel encoders, so the forward speed is the speed that the pulses to the motor driver ask for, passed through a model of the driver's lag and the acceleration limit.

The first heading is the `initial_heading` parameter. Driving corrects it. On the real robot, start the mission with the robot on a known heading, or use a GNSS receiver with two antennas.

### Path tracking

`tracker.py` follows the path the way the planner costs it: stop, turn in place, drive.

- On a segment, a fuzzy controller (`fuzzy.py`, zero-order Sugeno, 5 x 5 rules) steers on the cross-track error and the heading error.
- A skid-steer robot does not turn at the rate it is told to. The tracker learns the ratio from the gyro and corrects its turn commands.
- The blade is on along the lanes. It stays on over moves between cells shorter than 3 m, and is off on longer moves.
- The robot waits for the blade to reach its speed before it enters a lane.
- A segment is done only when the robot reaches its end along the segment. When the robot is more than 0.6 m from the segment, it drives straight back to the point where it left it.

### Mission

`plan.py` calls the pipeline of `lawn_mower_planning` with the robot's cutting width. The mission pauses when the controller leaves automatic mode (operator takeover, lost radio link, emergency stop, bumper). When automatic mode returns, the robot drives back to the point where it left the path and continues. If the straight line back crosses an obstacle or leaves the lawn, the mission reports `LOST` and waits for the operator to drive the robot closer.

Every method of `lawn_mower_planning` plans the demo lawn, and `energy_ga` plans every shared lawn at the default clearance. Only `demo` and `two_beds` were driven in Gazebo.

## Clearance and coverage

The default clearance keeps the whole body inside the lawn, also while the robot turns in place. For the faculty robot estimate this is 0.89 m, but the blade cuts only 0.25 m to each side. The robot thus leaves a strip of about 0.64 m along every edge and obstacle. On the 12 m x 9 m demo lawn, the planned coverage is 61.5 %.

A smaller `clearance` raises the coverage, and lets the corners of the body swing over the lawn edge in the turns. The evaluator reports how far (`footprint_excursion_m`).

## Safety check

With the simulation running, play the operator and check the reactions:

```bash
ros2 launch lawn_mower_navigation coverage_sim.launch.py headless:=true real_time_factor:=2
ros2 run lawn_mower_navigation safety_check --ros-args -p use_sim_time:=true
```

The check takes over by radio, switches the transmitter off and presses the emergency stop. It prints one line for each step and exits with 1 if a step fails.

The bumper scenario drives the robot into the flower bed of the demo lawn in manual mode. It checks that the bed stops the robot, that the bumper reports the contact, and that automatic mode does not start against the obstacle:

```bash
ros2 launch lawn_mower_navigation coverage_sim.launch.py headless:=true real_time_factor:=2 auto_after:=-1.0
ros2 run lawn_mower_navigation safety_check --ros-args -p use_sim_time:=true -p scenario:=bumper
```

## Reference run

`faculty_robot_estimate`, `energy_ga`, seed 0, headless, `real_time_factor:=3`. The demo run adds `budget:=300`; the `two_beds` run uses `lawn:=two_beds`.

| Result | `demo` (12 m x 9 m, one bed) | `two_beds` (20 m x 12 m, two beds) |
| --- | --- | --- |
| Coverage, simulated and planned | 61.6 % and 61.5 % | 65.0 % and 63.8 % |
| Duration and distance | 472 s and 158 m | 917 s and 332 m |
| Tracking error, mean and largest | 0.02 m and 0.16 m | 0.01 m and 0.15 m |
| Localisation error, mean and largest | 0.02 m and 0.07 m | 0.01 m and 0.07 m |
| Body outside the lawn | 0.00 m | 0.00 m |
| Energy: traction, blade, auxiliary | 5.4 Wh, 59.7 Wh, 2.0 Wh | 10.1 Wh, 120.4 Wh, 3.8 Wh |

The simulated coverage can be above the planned one, because the blade stays on over short moves between cells.

These numbers use the estimate config. They show that the simulation works. They are not results for the paper.

## Limits

- The ground is flat. The grass is not simulated: the blade draws its full cutting power whenever it spins.
- The robot senses an obstacle only when its bumper touches it. It has no lidar or camera. It stays clear of the obstacles in the lawn polygon.
- The bumper covers the chassis, not the wheels, which stand out to the sides.
- The simulation was run without the Gazebo window (`headless:=true`) only.
- The electronics are behavioural models, not the firmware of the real parts.
