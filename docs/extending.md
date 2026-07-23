# Extending The Cartesian Command Stack

This guide explains where to edit when adding a new input, behaviour state, or geometric state.

The main design rule is simple: every command source should publish a `geometry_msgs/msg/TwistStamped`. Device-specific logic, such as joystick axes and buttons, belongs outside `cartesian_command_manager`.

## Package Boundaries

`cartesian_command_manager` owns:

- Input freshness and command combination.
- State parsing and state machine validation.
- Geometric command shaping.
- Final `TwistStamped` output.

`joystick_command_mapper` owns:

- `sensor_msgs/msg/Joy` parsing.
- Axis mapping.
- Deadzone and norm scaling.
- Joystick velocity saturation.
- Button-to-state mapping.

If a new source already produces Cartesian velocity, connect it directly to the manager. If it starts from a device-specific message, create or update a mapper node outside the manager.

## Current Command Flow

1. A command source publishes `geometry_msgs/msg/TwistStamped`.
2. `CartesianCommandManager` converts the ROS message to `manager_core::CartesianCommand`.
3. `InputManager` stores the command with a timestamp.
4. On each update, `InputManager::getFullCommand()` averages all enabled, non-timeout commands with positive weight.
5. `CommandPipeline::update()` applies the active behaviour.
6. If the behaviour allows it, `CommandPipeline::update()` applies the current geometric state.
7. The manager publishes the final `geometry_msgs/msg/TwistStamped`.

Current multi-input handling is in:

- `include/cartesian_command_manager/core/input_manager.hpp`
- `src/core/input_manager.cpp`

The current code has a `weight` field in `InputChannel`, but there is no parameter exposed yet to configure input weights.

## Add A New Input

Use this path when the new source publishes a Cartesian velocity command, for example visual servoing, keyboard teleop, autonomy, or an external planner.

Current note: `InputSource::VISUAL_SERVOING` and `inputs.visual_servoing` already exist, but the ROS subscription and callback are not wired yet. If you are adding visual servoing, reuse those existing names instead of adding another enum and parameter group.

### 1. Add The Source Enum

Edit `include/cartesian_command_manager/core/types.hpp`:

```cpp
enum class InputSource
{
  JOYSTICK,
  VISUAL_SERVOING,
  MY_NEW_INPUT
};
```

### 2. Add Parameters

Edit `src/cartesian_command_manager_parameters.yaml`.

Add a topic under `topics`:

```yaml
my_new_input_command:
  type: string
  default_value: "/my_new_input_cartesian_command"
  description: "Topic carrying my new Cartesian input command."
```

Add input settings under `inputs`:

```yaml
my_new_input:
  timeout_sec:
    type: double
    default_value: 0.2
    description: "Maximum command age before this input is ignored."
  enabled:
    type: bool
    default_value: true
    description: "Whether this input is enabled at startup."
```

The parameter header is generated at build time by `generate_parameter_library`.

### 3. Add ROS Members

Edit `include/cartesian_command_manager/ros/cartesian_command_manager.hpp`.

Add a topic string:

```cpp
std::string my_new_input_command_topic_;
```

Add a subscription:

```cpp
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr my_new_input_sub_;
```

Add a callback declaration:

```cpp
void myNewInputCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
```

### 4. Read Parameters And Register The Channel

Edit `src/ros/cartesian_command_manager.cpp` in `readParameters()`:

```cpp
my_new_input_command_topic_ = params_.topics.my_new_input_command;

pipeline_.addInputChannel(
    manager_core::InputSource::MY_NEW_INPUT,
    params_.inputs.my_new_input.timeout_sec,
    params_.inputs.my_new_input.enabled);
```

### 5. Subscribe To The Topic

Edit `setupSubscribers()`:

```cpp
my_new_input_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    my_new_input_command_topic_, 10,
    std::bind(&CartesianCommandManager::myNewInputCallback, this, std::placeholders::_1));
```

### 6. Store Incoming Commands

Add the callback implementation:

```cpp
void CartesianCommandManager::myNewInputCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  manager_core::CartesianCommand command;
  command.linear = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
  command.angular = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
  pipeline_.setInputCommand(manager_core::InputSource::MY_NEW_INPUT, command, now().seconds());
}
```

### 7. Update Runtime Config

Edit `bringup/config/explorer_params.yaml` with matching topic and input settings:

```yaml
cartesian_command_manager:
  ros__parameters:
    topics:
      my_new_input_command: "/my_new_input_cartesian_command"
    inputs:
      my_new_input:
        timeout_sec: 0.2
        enabled: true
```

### 8. Test The Input

Build:

```bash
colcon build --packages-select cartesian_command_manager
source install/setup.bash
```

Publish a test command:

```bash
ros2 topic pub /my_new_input_cartesian_command geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Check output:

```bash
ros2 topic echo /cartesian_command
```

## Add A New Behaviour State

Use a behaviour state for policy-level command handling, for example choosing how to arbitrate between human input and an autonomous input.

Important: adding the enum and parser only makes the state selectable. You must also implement what the behaviour does in the pipeline.

The current strategy is behaviour first, then geometric shaping only for behaviours where that is meaningful. `PASSTHROUGH` and `SHARED` use geometric shaping. `HOMING` bypasses geometric shaping because it generates an autonomous command and should not be accidentally broken by `rotation`, `translation`, `snake`, or another user-selected geometric mode.

`HOMING` also owns its completion transition: when the configured joint error is inside tolerance, the manager switches back to `PASSTHROUGH` and republishes `passthrough` on the behaviour topic so the joystick mapper stays synchronized.

### 1. Add The Enum

Edit `include/cartesian_command_manager/core/types.hpp`:

```cpp
enum class BehaviourState
{
  PASSTHROUGH,
  SHARED,
  HOMING,
  MY_BEHAVIOUR
};
```

### 2. Parse The String

Edit `parseBehaviourState()` in `src/ros/cartesian_command_manager.cpp`:

```cpp
if (normalized == "my_behaviour")
{
  return manager_core::BehaviourState::MY_BEHAVIOUR;
}
```

State strings should be lowercase snake case. The manager normalizes case and converts `-` to `_`.

### 3. Validate State Combinations

Edit `src/core/state_machine.cpp` if some behaviour and geometric state combinations should be rejected:

```cpp
bool StateMachine::isCombinationAllowed(BehaviourState bState, GeometricState gState) const
{
  if (bState == BehaviourState::MY_BEHAVIOUR && gState == GeometricState::SNAKE)
  {
    return false;
  }
  return true;
}
```

### 4. Implement The Behaviour

Edit `src/core/command_pipeline.cpp`.

Read the active behaviour with:

```cpp
const auto behaviour_state = state_machine_.behaviour_state();
```

Then apply the behaviour before or after geometric shaping, depending on the intended semantics.

Examples:

- Input arbitration belongs before geometric shaping.
- Final safety filters usually belong after geometric shaping.
- Mode-specific blending may need both the behaviour state and the active input sources.

### 5. Add Joystick Selection If Needed

Only update `joystick_command_mapper` if the new behaviour should be selectable from joystick buttons.

Typical changes in the mapper package:

- Add a parameter in its parameter YAML or declared parameters.
- Add a button index member.
- Call the existing behaviour-button handling with the new state string.
- Add the button to the shared launch config in `bringup/config/explorer_params.yaml`.

### 6. Test The Behaviour

```bash
ros2 topic pub --once /behaviour_state std_msgs/msg/String "{data: my_behaviour}"
ros2 topic echo /cartesian_command
```

Also test an invalid string:

```bash
ros2 topic pub --once /behaviour_state std_msgs/msg/String "{data: typo_state}"
```

The manager should warn and keep its previous state.

## Add A New Geometric State

Use a geometric state when the command shape changes, for example translation-only, rotation-only, or a Jacobian-based mapping.

### 1. Add The Enum

Edit `include/cartesian_command_manager/core/types.hpp`:

```cpp
enum class GeometricState
{
  ROTATION,
  TRANSLATION,
  BOTH,
  JACO,
  SNAKE,
  MY_GEOMETRIC_STATE
};
```

### 2. Parse The String

Edit `parseGeometricState()` in `src/ros/cartesian_command_manager.cpp`:

```cpp
if (normalized == "my_geometric_state")
{
  return manager_core::GeometricState::MY_GEOMETRIC_STATE;
}
```

### 3. Decide Between Inline Logic And A Shaper

Use inline logic in `CommandPipeline::update()` for simple filters:

```cpp
case GeometricState::MY_GEOMETRIC_STATE:
  output.angular.setZero();
  output.linear.z() = 0.0;
  break;
```

Use a shaper class when the state needs memory, parameters, Jacobians, pose, or more than a few lines of logic.

### 4. Add A Shaper Class

Create files similar to the existing shapers:

- `include/cartesian_command_manager/core/shapers/geometric/my_geometric_state.hpp`
- `src/core/shapers/geometric/my_geometric_state.cpp`

Implement the `Shaper` interface:

```cpp
class MyGeometricStateShaper : public Shaper
{
public:
  CartesianCommand update(const CartesianCommand &input,
                          const RobotContext &context,
                          double dt_sec) override;
};
```

Use `RobotContext` for end-effector pose, velocity, and Jacobian:

```cpp
context.ee_pose
context.ee_vel
context.ee_jac
```

### 5. Register The Shaper

Edit `include/cartesian_command_manager/core/command_pipeline.hpp` to include the new shaper header.

Edit `src/core/command_pipeline.cpp` in `CommandPipeline::configure()`:

```cpp
registerGeometricShaper(
    GeometricState::MY_GEOMETRIC_STATE,
    std::make_unique<MyGeometricStateShaper>());
```

Add the source file to `CMakeLists.txt` under `${PROJECT_NAME}_core`.

### 6. Add Parameters If Needed

If the shaper needs parameters:

1. Add a config struct in `include/cartesian_command_manager/core/command_pipeline.hpp`.
2. Add parameters in `src/cartesian_command_manager_parameters.yaml`.
3. Copy generated ROS parameters into `CommandPipelineConfig` in `readParameters()`.
4. Pass the config into the shaper in `CommandPipeline::configure()`.
5. Add runtime values in `bringup/config/explorer_params.yaml`.

Follow the existing `snake.gain` path as the example.

### 7. Add Joystick Selection If Needed

Only update `joystick_command_mapper` if the new geometric state should be selectable from joystick buttons.

Typical mapper changes:

- Add a button index parameter.
- Add the button index member.
- Call the existing geometric-button handling with the new lowercase state string.
- Subscribe to the same state topic so the mapper stays synchronized with external state changes.
- Reject unknown incoming state strings.

### 8. Test The State

```bash
colcon build --packages-select joystick_command_mapper cartesian_command_manager
source install/setup.bash
ros2 topic pub --once /geometric_state std_msgs/msg/String "{data: my_geometric_state}"
ros2 topic echo /cartesian_command
```

Test direct command shaping with a known input:

```bash
ros2 topic pub /joystick_cartesian_command geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.01, y: 0.02, z: 0.03}, angular: {x: 0.1, y: 0.2, z: 0.3}}}"
```

## State Message Contract

State topics use `std_msgs/msg/String`.

Current accepted strings:

```text
geometric: both, translation, rotation, jaco, snake
behaviour: passthrough, shared, homing, go_home
```

The manager rejects unknown strings and keeps the previous state.

The joystick mapper should do the same for incoming state strings. Do not store unknown state names, because that can desynchronize mapper state from manager state.

Current toggle behaviour:

- Publishing the active geometric state again resets the manager to `both`.
- Publishing the active behaviour state again resets the manager to `passthrough`.

This is convenient for joystick buttons. If direct set semantics are needed later, change this in:

- `CartesianCommandManager::geometricStateCallback()`
- `CartesianCommandManager::behaviourStateCallback()`

## Review Checklist

Before trying a new extension on hardware:

- Build both packages cleanly.
- Verify every new parameter exists in both generated parameter YAML and runtime config.
- Verify launch node names match the top-level YAML keys.
- Echo the input command topic and confirm timestamps are updating.
- Echo `/cartesian_command` and confirm timeout produces zero or empty command output as expected.
- Test every state string manually with `ros2 topic pub --once`.
- Test typo strings and confirm the manager warns and keeps the previous state.
- Start in simulation with low velocity limits.
