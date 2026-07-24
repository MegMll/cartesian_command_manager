#include "cartesian_command_manager/core/shapers/behaviour/homing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace manager_core
{
  namespace
  {
    void limitVectorNorm(Eigen::VectorXd &vector, double max_norm)
    {
      if (max_norm <= 0.0)
      {
        vector.setZero();
        return;
      }

      const auto norm = vector.norm();
      if (norm > max_norm)
      {
        vector *= max_norm / norm;
      }
    }

    void limitVectorNorm(Eigen::Vector3d &vector, double max_norm)
    {
      if (max_norm <= 0.0)
      {
        vector.setZero();
        return;
      }

      const auto norm = vector.norm();
      if (norm > max_norm)
      {
        vector *= max_norm / norm;
      }
    }
  } // namespace

  HomingBehaviour::HomingBehaviour(HomingBehaviourConfig config) : config_(std::move(config))
  {
  }

  std::optional<CartesianCommand> HomingBehaviour::update(const RobotContext &context) const
  {
    if (!hasValidConfig())
    {
      return std::nullopt;
    }

    const auto current_positions = orderedCurrentPositions(context);
    if (!current_positions)
    {
      return std::nullopt;
    }

    if (context.ee_jac.rows() != 6 ||
        context.ee_jac.cols() != static_cast<Eigen::Index>(config_.joint_names.size()))
    {
      return std::nullopt;
    }

    Eigen::VectorXd home_positions(config_.positions.size());
    for (std::size_t i = 0; i < config_.positions.size(); ++i)
    {
      home_positions(static_cast<Eigen::Index>(i)) = config_.positions[i];
    }

    const Eigen::VectorXd joint_error = home_positions - *current_positions;
    if (isTargetReached(context))
    {
      return CartesianCommand{};
    }

    Eigen::VectorXd joint_velocity = config_.gain * joint_error;
    limitVectorNorm(joint_velocity, config_.max_joint_velocity);

    const Eigen::VectorXd twist = context.ee_jac * joint_velocity;
    if (twist.size() != 6)
    {
      return std::nullopt;
    }

    CartesianCommand command;
    command.linear = twist.head<3>();
    command.angular = twist.tail<3>();

    limitVectorNorm(command.linear, config_.max_linear_velocity);
    limitVectorNorm(command.angular, config_.max_angular_velocity);

    return command;
  }

  bool HomingBehaviour::isTargetReached(const RobotContext &context) const
  {
    if (!hasValidConfig())
    {
      return false;
    }

    const auto current_positions = orderedCurrentPositions(context);
    if (!current_positions)
    {
      return false;
    }

    Eigen::VectorXd home_positions(config_.positions.size());
    for (std::size_t i = 0; i < config_.positions.size(); ++i)
    {
      home_positions(static_cast<Eigen::Index>(i)) = config_.positions[i];
    }

    const Eigen::VectorXd joint_error = home_positions - *current_positions;
    return joint_error.lpNorm<Eigen::Infinity>() <= config_.position_tolerance;
  }

  bool HomingBehaviour::hasValidConfig() const
  {
    return !config_.joint_names.empty() && !config_.joint_names.front().empty() &&
           config_.joint_names.size() == config_.positions.size() && config_.gain > 0.0 &&
           config_.position_tolerance >= 0.0;
  }

  std::optional<Eigen::VectorXd> HomingBehaviour::orderedCurrentPositions(
      const RobotContext &context) const
  {
    if (context.joint_names.empty() ||
        context.joint_positions.size() != static_cast<Eigen::Index>(context.joint_names.size()))
    {
      return std::nullopt;
    }

    Eigen::VectorXd positions(config_.joint_names.size());
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i)
    {
      const auto joint =
          std::find(context.joint_names.begin(), context.joint_names.end(), config_.joint_names[i]);
      if (joint == context.joint_names.end())
      {
        return std::nullopt;
      }

      const auto index = std::distance(context.joint_names.begin(), joint);
      positions(static_cast<Eigen::Index>(i)) = context.joint_positions(index);
    }

    return positions;
  }
} // namespace manager_core
