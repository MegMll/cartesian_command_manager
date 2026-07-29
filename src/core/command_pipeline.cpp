#include "cartesian_command_manager/core/command_pipeline.hpp"

namespace manager_core
{

  void CommandPipeline::configure(const CommandPipelineConfig &config)
  {
    geometric_shapers_.clear();

    registerGeometricShaper(GeometricState::JACO, std::make_unique<JacoShaper>(config.jaco));
    registerGeometricShaper(GeometricState::SNAKE, std::make_unique<SnakeShaper>(config.snake));
    joint_target_behaviour_ = JointTargetBehaviour(config.joint_targets);
  }

  void CommandPipeline::addInputChannel(InputSource source, double timeout_sec, bool enabled)
  {
    input_manager_.addInputChannel(source, timeout_sec, enabled);
  }

  void CommandPipeline::enableInput(InputSource source)
  {
    input_manager_.enableInputChannel(source);
  }

  void CommandPipeline::disableInput(InputSource source)
  {
    input_manager_.disableInputChannel(source);
  }

  void CommandPipeline::setBehaviourState(BehaviourState state)
  {
    behaviour_state_ = state;
  }

  void CommandPipeline::setGeometricState(GeometricState state)
  {
    geometric_state_ = state;
  }

  bool CommandPipeline::setJointTarget(const std::string &target_name)
  {
    return joint_target_behaviour_.setTarget(target_name);
  }

  void CommandPipeline::setInputCommand(InputSource source, const CartesianCommand &command,
                                        double stamp_sec)
  {
    input_manager_.setCommand(source, command, stamp_sec);
  }

  std::optional<CartesianCommand> CommandPipeline::update(double now_sec, double dt_sec,
                                                          const RobotContext &context)
  {
    auto command = applyBehaviour(now_sec, context);
    if (!command)
    {
      return std::nullopt;
    }

    CartesianCommand output = *command;
    if (behaviour_state_ == BehaviourState::JOINT_TARGET)
    {
      return output;
    }

    applyGeometricState(output, context, dt_sec);
    return output;
  }

  std::optional<CartesianCommand> CommandPipeline::applyBehaviour(double now_sec,
                                                                  const RobotContext &context) const
  {
    switch (behaviour_state_)
    {
    case BehaviourState::PASSTHROUGH:
      return input_manager_.getFullCommand(now_sec);
    case BehaviourState::JOINT_TARGET:
      return joint_target_behaviour_.update(context);
    }

    return std::nullopt;
  }

  void CommandPipeline::applyGeometricState(CartesianCommand &output, const RobotContext &context,
                                            double dt_sec)
  {
    switch (geometric_state_)
    {
    case GeometricState::BOTH:
      break;
    default:
      applyRegisteredGeometricShaper(geometric_state_, output, context, dt_sec);
      break;
    }
  }

  std::vector<InputSource> CommandPipeline::getValidInputSources(double now_sec) const
  {
    return input_manager_.getValidSources(now_sec);
  }

  bool CommandPipeline::isJointTargetReached(const RobotContext &context) const
  {
    return joint_target_behaviour_.isComplete(context);
  }

  std::optional<std::string> CommandPipeline::validateJointTarget(
      const RobotContext &context) const
  {
    return joint_target_behaviour_.validate(context);
  }

  void CommandPipeline::registerGeometricShaper(GeometricState state,
                                                std::unique_ptr<Shaper> shaper)
  {
    if (!shaper)
    {
      return;
    }

    geometric_shapers_[state] = std::move(shaper);
  }

  bool CommandPipeline::applyRegisteredGeometricShaper(GeometricState state,
                                                       CartesianCommand &command,
                                                       const RobotContext &context, double dt_sec)
  {
    const auto shaper = geometric_shapers_.find(state);
    if (shaper == geometric_shapers_.end())
    {
      return false;
    }

    command = shaper->second->update(command, context, dt_sec);
    return true;
  }
} // namespace manager_core
