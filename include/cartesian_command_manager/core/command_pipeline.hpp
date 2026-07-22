#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "cartesian_command_manager/core/input_manager.hpp"
#include "cartesian_command_manager/core/state_machine.hpp"
#include "cartesian_command_manager/core/types.hpp"
#include "cartesian_command_manager/core/shapers/shaper.hpp"
#include "cartesian_command_manager/core/shapers/geometric/jaco.hpp"
#include "cartesian_command_manager/core/shapers/geometric/snake.hpp"

namespace manager_core
{
  struct CommandPipelineConfig
  {
    SnakeShaperConfig snake;
  };

  class CommandPipeline
  {
  public:
    void configure(const CommandPipelineConfig &config);

    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);

    void enableInput(InputSource source);
    void disableInput(InputSource source);

    bool setBehaviourState(BehaviourState state);
    bool setGeometricState(GeometricState state);

    void setInputCommand(InputSource source, const CartesianCommand &command, double stamp_sec);

    std::optional<CartesianCommand> update(double now_sec, double dt_sec,
                                           const RobotContext &context);

    std::vector<InputSource> getValidInputSources(double now_sec) const;

  private:
    void registerGeometricShaper(GeometricState state, std::unique_ptr<Shaper> shaper);
    bool applyRegisteredGeometricShaper(GeometricState state, CartesianCommand &command,
                                        const RobotContext &context, double dt_sec);

    InputManager input_manager_;
    StateMachine state_machine_;

    std::unordered_map<GeometricState, std::unique_ptr<Shaper>> geometric_shapers_;
  };
} // namespace manager_core
