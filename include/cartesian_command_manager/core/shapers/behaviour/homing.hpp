#pragma once

#include <optional>
#include <string>
#include <vector>

#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  struct HomingBehaviourConfig
  {
    std::vector<std::string> joint_names;
    std::vector<double> positions;
    double gain{1.0};
    double max_joint_velocity{0.2};
    double max_linear_velocity{0.05};
    double max_angular_velocity{0.1};
    double position_tolerance{0.01};
  };

  class HomingBehaviour
  {
  public:
    explicit HomingBehaviour(HomingBehaviourConfig config = {});

    std::optional<CartesianCommand> update(const RobotContext &context) const;
    bool isTargetReached(const RobotContext &context) const;

  private:
    bool hasValidConfig() const;
    std::optional<Eigen::VectorXd> orderedCurrentPositions(const RobotContext &context) const;

    HomingBehaviourConfig config_;
  };
} // namespace manager_core
