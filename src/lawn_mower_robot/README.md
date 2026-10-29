# lawn_mower_robot

The robot as the rest of the project sees it: one config file per robot, a loader, the skid-steer kinematic and energy models, and the Gazebo launch.

The paper models the faculty robot. Until its measurements arrive, the **Pioneer 3-AT** is the placeholder. Its energy numbers are pipeline checks, not paper results.

## Robot configs

| File | Robot |
| --- | --- |
| `config/robot/pioneer3at.yaml` | Pioneer 3-AT, with a source for every value. |
| `config/robot/faculty_robot.yaml` | Faculty robot. A stub with `TODO` values. The header lists the measurements needed. |

Each value has a `value`, a `unit` and a `source` key from the file's `sources` list:

```yaml
track_width: {value: 0.394, unit: m, source: amr_urdf, note: "..."}
```

`lawn_mower_robot.robot_config.load_robot_config()` reads a file and fails with every problem listed: a missing or unknown field, a `TODO` value, a wrong unit or a source that is not in `sources`. The schema is `SCHEMA` in `robot_config.py`. Values with `source: assumption` are logged as warnings each time a node loads the file.

To use another robot, copy `pioneer3at.yaml`, fill in the values and pass the file:

```bash
ros2 launch lawn_mower_robot sim.launch.py robot:=faculty_robot
ros2 launch lawn_mower_robot sim.launch.py robot:=/path/to/my_robot.yaml
```

New nodes read the robot with:

```python
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
cfg = load_robot_config(resolve_robot_config(robot))  # robot: name or path
cfg.geometry.track_width
```

## Models

| Module | Content |
| --- | --- |
| `kinematics.py` | Skid-steer kinematics with the ICR expansion factor chi and slip s (paper Sec. 3.2). |
| `energy.py` | Rolling, turning and acceleration energy per segment, and an integrator for measured velocities (paper Sec. 3.3). |
| `sdf_model.py` | Builds the Gazebo SDF model from the config. |

The models use only config symbols. `test/test_no_robot_literals.py` fails if a number that could be a robot value appears in the code.

## Gazebo

`sim.launch.py` builds the robot from the config, so the simulated geometry, mass, inertia, friction and speed limits match the models. The downloaded Pioneer 3-AT Gazebo model gives the visual meshes (`gazebo.chassis_mesh` and `gazebo.wheel_mesh`). Pass the folder that contains the `pioneer3at` model folder as `model_path`:

```bash
ros2 launch lawn_mower_robot sim.launch.py model_path:=~/gazebo_models
```

Without the meshes the robot is drawn as a box and four cylinders. The drive is Gazebo's DiffDrive system with two wheels per side and the effective track width chi B. Topics: `/cmd_vel`, `/odom` (wheel odometry), `/ground_truth/odom` and `/joint_states`.

## Smoke test

Drive a straight line and in-place turns, and log the energy of each segment:

```bash
ros2 launch lawn_mower_robot smoke_test.launch.py model_path:=~/gazebo_models energy_log:=$HOME/energy_segments.csv
```

The CSV has one row per segment: target and measured distance or angle, duration, rolling, turning, kinetic and auxiliary energy, the battery energy in J and Wh, and the model's closed-form prediction. Add `headless:=true` to run without the Gazebo window.

To print the SDF of a config:

```bash
ros2 run lawn_mower_robot robot_sdf pioneer3at
```
