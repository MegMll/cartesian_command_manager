# Cartesian Command Manager

`cartesian_command_manager` combines Cartesian velocity inputs, applies the selected command shaping state, and publishes a single `geometry_msgs/msg/TwistStamped` command for the Cartesian velocity controller.

The joystick-specific mapping is intentionally kept outside this package in `joystick_command_mapper`. The manager treats joystick commands like any other input: a timestamped Cartesian twist.

## Runtime Graph

Typical explorer setup:

```text
/joy
  -> joystick_command_mapper
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

`joystick_command_mapper`:

- Subscribes to `sensor_msgs/msg/Joy`.
- Applies joystick deadzone.
- Publishes a `geometry_msgs/msg/TwistStamped` command.
- Publishes state requests as `std_msgs/msg/String`.

## Current Topics

Configured in `bringup/config/explorer_params.yaml`:

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Joystick Cartesian velocity command |
| `/geometric_state` | `std_msgs/msg/String` | input | Geometric state request |
| `/behaviour_state` | `std_msgs/msg/String` | input | Behaviour state request |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` | input | Current end-effector pose |
| `/ee_velocity` | `geometry_msgs/msg/TwistStamped` | input | Current end-effector velocity |
| `/ee_jac` | `std_msgs/msg/Float64MultiArray` | input | Current end-effector Jacobian |
| `/joint_states` | `sensor_msgs/msg/JointState` | input | Current joint positions for homing |
| `/cartesian_command` | `geometry_msgs/msg/TwistStamped` | output | Final command sent to the velocity controller |

## States

Geometric states accepted by the manager:

- `both`: keep linear and angular velocity.
- `translation`: zero angular velocity.
- `rotation`: zero linear velocity.
- `jaco`: apply the Jaco geometric shaper.
- `snake`: apply the Snake geometric shaper.

Behaviour states accepted by the manager:

- `passthrough`: default behaviour.
- `homing` or `go_home`: generate an autonomous Cartesian command that drives configured joints toward the configured home positions.

Startup defaults:

- Geometric state: `both`
- Behaviour state: `passthrough`

Homing configuration is under `behaviours.homing` in `bringup/config/explorer_params.yaml`. The configured joint order must match the Jacobian column order.

When homing reaches `position_tolerance`, the manager publishes one zero command and automatically switches behaviour back to `passthrough`.

Repeated state requests currently behave like toggles:

- Requesting the active geometric state returns to `both`.
- Requesting the active behaviour state returns to `passthrough`.

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select joystick_command_mapper cartesian_command_manager
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

For first tests, use simulation, conservative velocity limits, and an external stop path. The final velocity limits are applied by `qontrol_controller`; joystick deadzone is configured under `joystick_command_mapper.ros__parameters.deadzone`.

The joystick homing button is configured with `homing_button_index`. It is disabled by default with `-1`; assign a real button index only after the homing joint target has been checked in simulation.

## Manual Checks

Publish a direct Cartesian command:

```bash
ros2 topic pub /joystick_cartesian_command geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Change geometric state:

```bash
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: translation}"
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: rotation}"
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: both}"
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
