# Cartesian Command Manager

`cartesian_command_manager` combines Cartesian velocity inputs, applies the selected command shaping state, and publishes a single `geometry_msgs/msg/TwistStamped` command for the Cartesian velocity controller.

The joystick-specific mapping is intentionally kept outside this package in `joystick_mapper`. The manager treats joystick commands like any other input: a timestamped Cartesian twist.

## Runtime Graph

Typical explorer setup:

```text
/joy
  -> joystick_mapper
  -> /joystick_cartesian_command
  -> cartesian_command_manager
  -> /cartesian_command
  -> qontrol_explorer
```

State buttons from the joystick mapper publish string commands on:

```text
/geometric_state
/behaviour_state
```

The manager also subscribes to robot context topics used by geometric shapers:

```text
/ee_pose
/ee_velocity
/ee_jac
/joint_states
```

## Responsibilities

`cartesian_command_manager`:

- Subscribes to Cartesian input commands as `geometry_msgs/msg/TwistStamped`.
- Tracks input freshness with per-input timeouts.
- Combines all currently valid input commands.
- Parses geometric and behaviour state strings.
- Applies geometric command shaping.
- Transforms input commands to the configured output frame before combining them.
- Publishes the final `geometry_msgs/msg/TwistStamped` command.

Current implementation note: the joystick input is wired end to end. `VISUAL_SERVOING` and its parameters exist as the next input source, but the ROS subscription and callback are not implemented yet.

`joystick_mapper`:

- Subscribes to `sensor_msgs/msg/Joy`.
- Applies joystick deadzone.
- Publishes a `geometry_msgs/msg/TwistStamped` command.
- Publishes state requests as `std_msgs/msg/String`.

## Current Topics

Manager topics are configured in `bringup/config/explorer_params.yaml`.
Joystick mapper topics must match them and are configured in
`joystick_mapper/config/joystick_3d.yaml`.

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Joystick Cartesian velocity command |
| `/geometric_state` | `std_msgs/msg/String` | input | Geometric state request |
| `/behaviour_state` | `std_msgs/msg/String` | input | Behaviour state request |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` | input | Current end-effector pose |
| `/ee_velocity` | `geometry_msgs/msg/TwistStamped` | input | Current end-effector velocity |
| `/ee_jac` | `std_msgs/msg/Float64MultiArray` | input | Current end-effector Jacobian |
| `/joint_states` | `sensor_msgs/msg/JointState` | input | Current joint positions for joint-target behaviours |
| `/cartesian_command` | `geometry_msgs/msg/TwistStamped` | output | Final command sent to the velocity controller |

## States

Geometric states accepted by the manager:

- `both`: no geometric shaper.
- `jaco`: apply the Jaco geometric shaper.
- `snake`: apply the Snake geometric shaper.

The Jaco shaper disables its generated angular velocity inside
`shapers.jaco.min_radius` and clamps generated `angular.z` to
`shapers.jaco.max_angular_velocity`.

Joystick-local axis layouts, such as B1/B2 mappings, are handled by
`joystick_mapper` before the command reaches this manager.

Behaviour requests accepted by the manager:

- `passthrough`: default behaviour.
- `homing` or `go_home`: drive configured joints toward the `home` joint target.
- `go_<target>`: drive configured joints toward another configured joint target.

Startup defaults:

- Geometric state: `both`
- Behaviour state: `passthrough`

Joint target configuration is under `behaviours.joint_targets` in
`bringup/config/explorer_params.yaml`. The configured joint order must match the
Jacobian column order. Add targets by appending to `target_names` and appending
one full joint-position block to `positions`.

Example with two six-joint targets:

```yaml
behaviours:
  joint_targets:
    joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    target_names: ["home", "ready"]
    positions: [
      2.5, 0.3, -2.4, 2.97, 1.2, -0.5,
      0.0, 0.4, -1.8, 2.2, 1.0, 0.0
    ]
```

The manager accepts joint-target requests only when the target config, joint
state, and Jacobian are valid. If context becomes invalid while a target is
active, the manager aborts the behaviour and publishes `passthrough` so the
joystick mapper stays synchronized.

When a joint target reaches `position_tolerance`, the manager publishes one zero command and automatically switches behaviour back to `passthrough`.

State requests are direct set commands. Publishing `jaco` selects `jaco`,
publishing `snake` selects `snake`, publishing `both` clears the geometric
shaper, and publishing `passthrough` clears the active joint-target behaviour.

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select joystick_mapper cartesian_command_manager
source install/setup.bash
```

If `colcon` fails with `ModuleNotFoundError: No module named 'catkin_pkg'`, the active Python environment is missing ROS build dependencies. Either build from an environment that has ROS Humble's Python packages, or install `catkin_pkg` into the active environment.

## Run

Simulation:

```bash
ros2 launch cartesian_command_manager explorer.launch.py use_simulation:=true
```

Hardware:

```bash
ros2 launch cartesian_command_manager explorer.launch.py use_simulation:=false
```

Use a specific joystick mapper config:

```bash
ros2 launch cartesian_command_manager explorer.launch.py \
  joystick_config_file:=$(ros2 pkg prefix joystick_mapper)/share/joystick_mapper/config/joystick_3d.yaml
```

For first tests, use simulation, conservative velocity limits, and an external stop path. The final velocity limits are applied by `qontrol_controller`; joystick deadzone is configured in `joystick_mapper/config/joystick_3d.yaml`.

The joystick homing button is configured with `homing_button_index` in the joystick mapper config. Assign a real button index only after the homing joint target has been checked in simulation.

## Manual Checks

Publish a direct Cartesian command:

```bash
ros2 topic pub /joystick_cartesian_command geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Change geometric state:

```bash
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: both}"
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: jaco}"
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: snake}"
```

Change behaviour state:

```bash
ros2 topic pub --once /behaviour_state std_msgs/msg/String "{data: go_home}"
ros2 topic pub --once /behaviour_state std_msgs/msg/String "{data: passthrough}"
```

Observe output:

```bash
ros2 topic echo /cartesian_command
```

## Extending

See [docs/extending.md](docs/extending.md) for the contributor tutorial covering:

- Adding a new input source.
- Adding a new behaviour state.
- Adding a new geometric state or geometric shaper.
