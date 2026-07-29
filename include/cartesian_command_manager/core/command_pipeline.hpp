#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "cartesian_command_manager/core/input_manager.hpp"
#include "cartesian_command_manager/core/shapers/behaviour/joint_target.hpp"
#include "cartesian_command_manager/core/shapers/geometric/jaco.hpp"
#include "cartesian_command_manager/core/shapers/geometric/snake.hpp"
#include "cartesian_command_manager/core/shapers/shaper.hpp"
#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  struct CommandPipelineConfig
  {
    JacoShaperConfig jaco;
    SnakeShaperConfig snake;
    JointTargetBehaviourConfig joint_targets;
  };

  class CommandPipeline
  {
  public:
    void configure(const CommandPipelineConfig &config);

    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);

    void enableInput(InputSource source);
    void disableInput(InputSource source);

    void setBehaviourState(BehaviourState state);
    void setGeometricState(GeometricState state);
    bool setJointTarget(const std::string &target_name);

    void setInputCommand(InputSource source, const CartesianCommand &command, double stamp_sec);

    std::optional<CartesianCommand> update(double now_sec, double dt_sec,
                                           const RobotContext &context);

    bool isJointTargetReached(const RobotContext &context) const;
    std::optional<std::string> validateJointTarget(const RobotContext &context) const;

    std::vector<InputSource> getValidInputSources(double now_sec) const;

  private:
    std::optional<CartesianCommand> applyBehaviour(double now_sec,
                                                   const RobotContext &context) const;
    void applyGeometricState(CartesianCommand &command, const RobotContext &context, double dt_sec);
    void registerGeometricShaper(GeometricState state, std::unique_ptr<Shaper> shaper);
    bool applyRegisteredGeometricShaper(GeometricState state, CartesianCommand &command,
                                        const RobotContext &context, double dt_sec);

    InputManager input_manager_;
    BehaviourState behaviour_state_{BehaviourState::PASSTHROUGH};
    GeometricState geometric_state_{GeometricState::BOTH};

    std::unordered_map<GeometricState, std::unique_ptr<Shaper>> geometric_shapers_;
    JointTargetBehaviour joint_target_behaviour_;
  };
} // namespace manager_core
