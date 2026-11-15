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
| `pattern` | `hybrid` | `hybrid`: lanes, then contour passes along the edges. `lanes`: lanes only. |
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
- A segment is done only when the robot reaches its end along the segment. When the robot is more than 0.6 m from the segment, it drives straight back to the point where it left it, if the body fits on that line and in the turns at its ends.

### Mission

Every method of `lawn_mower_planning` plans the demo lawn, and `energy_ga` plans every shared lawn in both patterns. Only `demo` and `two_beds` were driven in Gazebo.

`plan.py` calls the pipeline of `lawn_mower_planning` with the robot's cutting width. The mission pauses when the controller leaves automatic mode (operator takeover, lost radio link, emergency stop, bumper). When automatic mode returns, the robot drives back to the point where it left the path and continues. If the body does not fit on the straight line back, the mission reports `LOST` and waits for the operator to drive the robot closer.

## Hybrid coverage pattern

The robot turns in place at each lane end, so the lanes stay where a full turn fits: the free area, 0.89 m from every edge and obstacle for the faculty robot estimate. The blade cuts 0.25 m to each side. Lanes alone thus leave a strip of about 0.64 m along every edge.

The hybrid pattern (`pattern:=hybrid`, the default) drives the lanes first, and then contour passes that cut into that strip:

| Pass | Where the robot centre drives | Turns |
| --- | --- | --- |
| Turn ring | The boundary of the free area. | In place at each corner. A full turn fits. |
| Side ring | Half the body width plus a margin from the lawn edge (0.66 m) or from an obstacle (0.76 m). | The body fits there only along the edge. At a corner with no room to turn, the robot leaves the edge on a shallow diagonal, turns in the free area, and joins the next edge. |

`lawn_mower_planning.footprint.BodyCheck` checks every run and every turn of the contour passes against the lawn polygon. A piece where the body does not fit is left out.

| Margin | Lawn edge | Obstacle |
| --- | --- | --- |
| While the robot drives along an edge | 0.15 m | 0.25 m |
| At the worst moment of a turn or a diagonal | 0.05 m | 0.10 m |

An obstacle gets more room than the lawn edge. The edge is a line on the ground, the obstacle is solid, and the bumper does not cover the wheels. The margins are `Margins` in `plan.py`.

Planned coverage, `energy_ga`, `budget:=300`:

| Lawn | Lanes | Hybrid | Path length, lanes and hybrid |
| --- | --- | --- | --- |
| `demo` | 61.5 % | 77.2 % | 154 m and 260 m |
| `two_beds` | 63.8 % | 79.6 % | 326 m and 559 m |
| `backyard` | 77.8 % | 86.9 % | 701 m and 936 m |
| `l_shape` | 74.2 % | 85.8 % | 748 m and 1083 m |
| `three_beds` | 69.7 % | 83.3 % | 652 m and 989 m |
| `garden` | 84.0 % | 90.1 % | 1283 m and 1624 m |
| `park_trees` | 86.2 % | 91.4 % | 1890 m and 2260 m |
| `sports_edge` | 86.1 % | 92.0 % | 2459 m and 3008 m |
| `random_1` | 80.5 % | 89.0 % | 1042 m and 1357 m |
| `random_2` | 82.4 % | 89.4 % | 1097 m and 1384 m |
| `random_3` | 84.4 % | 90.7 % | 1072 m and 1311 m |

What stays uncut, and why:

- A strip of 0.41 m along the lawn edge and 0.51 m round each obstacle. The body is 1.02 m wide and the blade 0.5 m, so the blade cannot reach closer while the body stays inside.
- The corners of the lawn, where the side ring leaves the edge about 1.5 m before the corner.
- If the body may overhang the lawn edge on your site (the edge is a path, not a wall), lower `run_edge` and `pivot_edge` in `Margins`. This is the largest lever on coverage, and it is a fact of the site, not of the planner.

The contour passes overlap the lanes and each other, so the hybrid path is 20 % to 70 % longer than the lanes alone.

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

## Reference runs

`faculty_robot_estimate`, `energy_ga`, seed 0, headless, `real_time_factor:=3`. The demo runs add `budget:=300`.

| Result | `demo`, lanes | `demo`, hybrid | `two_beds`, lanes | `two_beds`, hybrid |
| --- | --- | --- | --- | --- |
| Coverage, simulated | 61.6 % | 77.4 % | 65.0 % | not run yet |
| Coverage, planned | 61.5 % | 77.2 % | 63.8 % | 79.6 % |
| Duration | 472 s | 767 s | 917 s | not run yet |
| Distance | 158 m | 267 m | 331 m | not run yet |
| Tracking error, mean and largest | 0.02 m and 0.16 m | 0.02 m and 0.13 m | 0.01 m and 0.15 m | not run yet |
| Body outside the lawn | 0.00 m | 0.01 m | 0.00 m | not run yet |
| Energy: traction, blade, auxiliary | 5.4, 59.7, 2.0 Wh | 8.5, 97.6, 3.2 Wh | 10.1, 120.5, 3.8 Wh | not run yet |

The simulated coverage of a lanes run can be above the planned one, because the blade stays on over short moves between cells.

These numbers use the estimate config. They show that the simulation works. They are not results for the paper.

## Limits

- The ground is flat. The grass is not simulated: the blade draws its full cutting power whenever it spins.
- The robot senses an obstacle only when its bumper touches it. It has no lidar or camera. It stays clear of the obstacles in the lawn polygon.
- The bumper covers the chassis, not the wheels, which stand out to the sides.
- The simulation was run without the Gazebo window (`headless:=true`) only.
- The electronics are behavioural models, not the firmware of the real parts.
