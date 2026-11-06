# FR3 ROS2/MoveIt Probing Architecture

This document is the current integration contract for moving the probing
project from MuJoCo-validated primitives to the Franka FR3 ROS2/MoveIt
pipeline.

## Current Default Route

Primary robot stack:

- Upstream workspace: `frankarobotics/franka_ros2`
- MoveIt package: `franka_fr3_moveit_config`
- Planning group: `fr3_arm`
- Base frame: `fr3_link0`
- Probing TCP: `spatula_tip`
- Controller: `fr3_arm_controller`
- One-shot probing backend mode: `move_group_action`
- Contact-stage planner: `pilz_industrial_motion_planner` / `LIN`

The Panda MoveIt demo package `moveit_resources_panda_moveit_config` is no
longer a valid primary route for ROS2 probing.

Start the default fake-hardware MoveIt stack in the container:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  use_spatula_tip:=true
```

Then run a probing entry:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=approach_smoke
```

`run_fr3_probe.launch.py` is the default fake-hardware test entry. It starts
only the probing node and connects to the already running `/move_group` and
`/controller_manager` graph.

## Entrypoints

The common implementation lives in `probing.ros2_probe_runner`.

`scripts/run_fr3_probe.py`

- Thin wrapper around `probing.ros2_probe_runner`.
- Default route for fake-hardware validation.
- Defaults to `tip_link:=spatula_tip`.

`scripts/run_real_robot_probe.py`

- Thin wrapper around the same runner.
- Kept as a compatibility entry for future real-hardware launch flows.
- Does not load Panda config and does not contain separate primitive logic.
- Defaults to real-robot preflight behavior:
  - `approach_z=0.55`
  - `allow_contact_primitives=false`
  - only `approach_smoke` is allowed unless explicitly unlocked

`scripts/run_real_robot_probe.launch.py`

- Connect-only FR3 launch for an already running FR3 MoveIt pipeline.
- No `moveit_resources_panda_moveit_config`.
- Defaults to `/move_action`, `fr3_arm`, `fr3_link0`, `spatula_tip`, and
  `fr3_arm_controller`.
- Supports `robot_ip`, `use_fake_hardware`, and `allow_contact_primitives`
  launch arguments. These are passed into the shared runner for run metadata
  and safety gating.
- Real-robot default is a high non-contact `approach_smoke` preflight.

## Backend Contract

`probing.backends.ros2_moveit_backend.ROS2MoveItBackend` owns the OMPL
free-space pose-goal, Pilz LIN, and fallback Cartesian-path transports.

Inputs:

- Cartesian pose goal or straight-line endpoint in `base_frame`
- Goal link, normally `spatula_tip`
- MoveIt planning group and controller
- Tolerances, planner ids, scaling factors, and action timeout

Outputs:

- A MoveItPy plan result when `backend_mode=moveit_py`
- A MoveGroup action result for free-space pose goals
- A Pilz `RobotTrajectory` for default contact stages
- A `GetCartesianPath` response for the fallback Cartesian mode

Default script behavior is `backend_mode=move_group_action`, which connects to
an existing `/move_action` server. This avoids changing behavior depending on
whether MoveItPy happens to be installed in the environment.

`APPROACH` uses the default OMPL free-space pose planner. Subsequent probing
stages default to Pilz LIN. Translation uses `0.010 m/s` cruise and
`0.050 m/s^2` acceleration; pure TILT/UNTILT uses `0.20 rad/s` and
`0.50 rad/s^2`. Pilz limits in `franka_fr3_moveit_config` are `0.10 m/s`,
`0.50 m/s^2`, and `2.0 rad/s`; each request derives MoveIt scaling factors
from the requested stage speed.

Pilz requests are plan-only first. The backend calls `/compute_fk` for every
joint-trajectory point and refuses execution when planned TCP line/position
error exceeds `0.1 mm` or planned cruise-speed error exceeds 2%. Only a passing
`RobotTrajectory` is sent to `/execute_trajectory`. After APPROACH, the actual
TCP orientation is read from TF and locked for translations. Inclined
insertion also locks rotation stages to the measured TCP position.

`contact_stage_mode=cartesian` retains the previous `/compute_cartesian_path`
implementation with custom FK-based trapezoidal retiming. It remains a
diagnostic fallback; `pose_goal` is the unconstrained legacy comparison route.

The backend does not implement:

- Real-time Cartesian velocity servoing
- Force control
- Contact aborts
- Sensor logging
- Sample-height estimation

Those belong above or beside the backend, not inside the pose-goal transport.

## Primitive Runner Contract

`probing.ros2_probe_runner` owns:

- Command-line arguments
- Primitive selection
- Stage sequence execution
- Fake-hardware safety limits
- Pose/orientation generation

Supported FR3 pose-stage entries:

| Primitive | Stage sequence |
| --- | --- |
| `approach_smoke` | `APPROACH` |
| `trajectory_smoke` | `APPROACH -> DOWN -> HOLD -> RETRACT` |
| `micro_compression` | `APPROACH -> COMPRESS -> HOLD -> RETRACT` |
| `horizontal_drag` | `APPROACH -> INSERT -> DRAG -> RETRACT` |
| `lift_detach` | `APPROACH -> COMPRESS -> HOLD -> LIFT` |
| `inclined_insertion` | `APPROACH -> TILT -> INSERT -> HOLD -> RETRACT -> UNTILT` |

Current hard limits:

- `compression_depth <= 0.03 m`
- `insert_depth <= 0.03 m`
- `inclined_insertion` tilt angle must be `10-15 deg`

These are fake-hardware trajectory limits. They are not sufficient for real
contact.

For the real-robot preflight entry, all primitives except `approach_smoke` are
blocked unless `allow_contact_primitives:=true` is passed explicitly. This also
blocks `trajectory_smoke`, because it contains a DOWN stage.

## Tool TCP Contract

The `spatula_tip` frame is the official probing TCP in the FR3 MoveIt path.

Current default transform:

```text
parent: fr3_hand_tcp
child:  spatula_tip
xyz:    0 0 0.2366
rpy:    0 0 0
```

This transform is a placeholder from the MuJoCo spatula approximation. Replace
it with measured calibration before any contact experiment.

Before running a primitive, verify:

```bash
ros2 param get /move_group robot_description_semantic
ros2 run tf2_ros tf2_echo fr3_link0 spatula_tip
ros2 control list_controllers
```

Expected controller state:

```text
joint_state_broadcaster active
fr3_arm_controller active
```

## Real Contact Controller Plan

MoveIt pose goals are used for non-contact approach. Pilz LIN paths with
trapezoidal constant-cruise timing are used for fake-hardware probing stages.
A precomputed JointTrajectory is still not sufficient for force-aware
contact insertion, compression, drag, or lift-detach on real hardware.

Planned contact-stage architecture:

1. Use MoveIt pose goals for non-contact approach.
2. Use the current Pilz LIN route for geometry and cruise-speed simulation
   checks, with the custom Cartesian implementation as fallback.
3. Switch to a constant-speed Cartesian controller for real contact stages.
4. Stream Cartesian velocity commands with a small bounded speed.
5. Read F/T samples at the control/logging rate.
6. Stop on any abort condition:
   - force norm exceeds threshold
   - force component exceeds directional threshold
   - travel exceeds stage distance
   - timeout
   - controller or sensor stale
7. Log every contact-stage sample with stage name, timestamp, pose if
   available, wrench, command, and stop reason.

The planned interface skeleton is in
`probing.controllers.cartesian_contact_controller`:

- `CartesianContactStageConfig`
- `CartesianContactStageResult`
- `CartesianServoBackend`
- `ConstantSpeedContactController`

The controller currently raises `NotImplementedError` and must not be used as a
hardware controller yet.

## F/T Logging Plan

The shared F/T interface is in `probing.sensors.ft_sensor`:

- `WrenchSample`
- `TipPoseSample`
- `ForceTorqueSensor`
- `CsvForceTorqueLogger`

The ROS2 stage logger is in `probing.sensors.ros2_probe_telemetry`:

- `ProbeTelemetryLogger`

Enable CSV telemetry from the FR3 launch:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=micro_compression \
  telemetry_log_path:=/tmp/fr3_micro_telemetry.csv
```

The logger records one row at each pose-stage start/done event and samples
during `HOLD`. The CSV includes:

- time
- primitive
- stage
- event
- commanded pose
- tip pose from TF, normally `fr3_link0 -> spatula_tip`
- latest wrench sample, when available

Current fake-hardware topic discovery:

```text
/franka_robot_state_broadcaster/external_wrench_in_base_frame
  type: geometry_msgs/msg/WrenchStamped
  QoS: BEST_EFFORT

/franka_robot_state_broadcaster/robot_state
  type: franka_msgs/msg/FrankaRobotState

/tf
  type: tf2_msgs/msg/TFMessage
```

`franka_robot_state_broadcaster` is declared in the FR3 controller config, but
with the current fake-hardware MoveIt launch it does not activate cleanly and
does not emit wrench/robot_state samples. Telemetry CSVs in fake hardware
therefore contain stage, commanded-pose, and TF tip-pose rows, with empty wrench
columns. On real hardware, activate/verify the broadcaster before contact
experiments.

Remaining implementation steps:

1. Validate wrench reception with an active real hardware or sensor publisher.
2. Normalize sensor frames into the probing TCP or a documented tool frame.
3. Calibrate bias before each trial.
4. Log raw and bias-corrected wrench samples.
5. Feed the same samples to contact abort logic.

## Real Robot Runbook

Use `docs/fr3_real_robot_migration_runbook.md` for the migration package,
spatula TCP change list, safety checklist, real-hardware bringup commands, and
contact primitive lockout policy.

## Optional/Future Work

MuJoCo remains useful for primitive development and qualitative validation, but
it is not the primary ROS2 control path.

Optional later work:

- MuJoCo plus `ros2_control` bridge for controller-in-the-loop simulation
- Cartesian servo benchmarking in simulation before FR3 hardware
- Unified sample-top estimation across MuJoCo and real sensors
- Trial metadata schema shared by MuJoCo CSVs and real ROS2 logs
