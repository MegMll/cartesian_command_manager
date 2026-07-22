#include "cartesian_command_manager/core/command_pipeline.hpp"

namespace manager_core
{

  void CommandPipeline::configure(const CommandPipelineConfig &config)
  {
    geometric_shapers_.clear();

    registerGeometricShaper(GeometricState::JACO, std::make_unique<JacoShaper>());
    registerGeometricShaper(GeometricState::SNAKE, std::make_unique<SnakeShaper>(config.snake));
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

  bool CommandPipeline::setBehaviourState(BehaviourState state)
  {
    return state_machine_.setState(state);
  }

  bool CommandPipeline::setGeometricState(GeometricState state)
  {
    return state_machine_.setState(state);
  }

  void CommandPipeline::setInputCommand(InputSource source, const CartesianCommand &command,
                                        double stamp_sec)
  {
    input_manager_.setCommand(source, command, stamp_sec);
  }

  std::optional<CartesianCommand> CommandPipeline::update(double now_sec, double dt_sec,
                                                          const RobotContext &context)
  {
    auto command = input_manager_.getFullCommand(now_sec);
    if (!command)
    {
      return std::nullopt;
    }

    CartesianCommand output = *command;
    const auto geometric_state = state_machine_.geometric_state();

    switch (geometric_state)
    {
    case GeometricState::TRANSLATION:
      output.angular.setZero();
      break;
    case GeometricState::ROTATION:
      output.linear.setZero();
      break;
    case GeometricState::BOTH:
      break;
    default:
      applyRegisteredGeometricShaper(geometric_state, output, context, dt_sec);
      break;
    }
    return output;
  }

  std::vector<InputSource> CommandPipeline::getValidInputSources(double now_sec) const
  {
    return input_manager_.getValidSources(now_sec);
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
