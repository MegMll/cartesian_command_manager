#pragma once

#include <optional>
#include <string>
#include <vector>

#include "cartesian_command_manager/core/shapers/behaviour/behaviour.hpp"

namespace manager_core
{
  struct JointTarget
  {
    std::string name;
    std::vector<double> positions;
  };

  struct JointTargetBehaviourConfig
  {
    std::vector<std::string> joint_names;
    std::vector<JointTarget> targets;
    double gain{1.0};
    double max_joint_velocity{0.2};
    double max_linear_velocity{0.05};
    double max_angular_velocity{0.1};
    double position_tolerance{0.01};
  };

  class JointTargetBehaviour : public Behaviour
  {
  public:
    explicit JointTargetBehaviour(JointTargetBehaviourConfig config = {});

    bool setTarget(const std::string &target_name);
    const std::string &activeTargetName() const;

    std::optional<CartesianCommand> update(const RobotContext &context) const override;
    std::optional<std::string> validate(const RobotContext &context) const override;
    bool isComplete(const RobotContext &context) const override;

  private:
    bool hasValidConfig() const;
    const JointTarget *targetByName(const std::string &target_name) const;
    const JointTarget *activeTarget() const;
    std::optional<Eigen::VectorXd> targetPositions(const JointTarget &target) const;
    std::optional<Eigen::VectorXd> orderedCurrentPositions(const RobotContext &context) const;

    JointTargetBehaviourConfig config_;
    std::string active_target_name_{"home"};
  };
} // namespace manager_core
