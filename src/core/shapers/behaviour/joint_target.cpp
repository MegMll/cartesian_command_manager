#include "cartesian_command_manager/core/shapers/behaviour/joint_target.hpp"

#include <algorithm>
#include <string>
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

  JointTargetBehaviour::JointTargetBehaviour(JointTargetBehaviourConfig config)
      : config_(std::move(config))
  {
    if (!config_.targets.empty() && !targetByName(active_target_name_))
    {
      active_target_name_ = config_.targets.front().name;
    }
  }

  bool JointTargetBehaviour::setTarget(const std::string &target_name)
  {
    if (!targetByName(target_name))
    {
      return false;
    }

    active_target_name_ = target_name;
    return true;
  }

  const std::string &JointTargetBehaviour::activeTargetName() const
  {
    return active_target_name_;
  }

  std::optional<CartesianCommand> JointTargetBehaviour::update(
      const RobotContext &context) const
  {
    const auto validation_error = validate(context);
    if (validation_error)
    {
      return std::nullopt;
    }

    const auto current_positions = orderedCurrentPositions(context);
    const auto target = activeTarget();
    if (!current_positions || target == nullptr)
    {
      return std::nullopt;
    }

    const auto target_positions = targetPositions(*target);
    if (!target_positions)
    {
      return std::nullopt;
    }

    const Eigen::VectorXd joint_error = *target_positions - *current_positions;
    if (isComplete(context))
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

  std::optional<std::string> JointTargetBehaviour::validate(const RobotContext &context) const
  {
    if (!hasValidConfig())
    {
      return "invalid joint-target behaviour configuration";
    }

    const auto target = activeTarget();
    if (target == nullptr)
    {
      return "unknown joint target '" + active_target_name_ + "'";
    }

    if (!targetPositions(*target))
    {
      return "joint target '" + target->name + "' has " +
             std::to_string(target->positions.size()) + " positions, expected " +
             std::to_string(config_.joint_names.size());
    }

    if (!orderedCurrentPositions(context))
    {
      return "joint state does not contain the configured joint-target joints";
    }

    if (context.ee_jac.rows() != 6 ||
        context.ee_jac.cols() != static_cast<Eigen::Index>(config_.joint_names.size()))
    {
      return "Jacobian shape is " + std::to_string(context.ee_jac.rows()) + "x" +
             std::to_string(context.ee_jac.cols()) + ", expected 6x" +
             std::to_string(config_.joint_names.size());
    }

    return std::nullopt;
  }

  bool JointTargetBehaviour::isComplete(const RobotContext &context) const
  {
    const auto current_positions = orderedCurrentPositions(context);
    const auto target = activeTarget();
    if (!current_positions || target == nullptr)
    {
      return false;
    }

    const auto target_positions = targetPositions(*target);
    if (!target_positions)
    {
      return false;
    }

    const Eigen::VectorXd joint_error = *target_positions - *current_positions;
    return joint_error.lpNorm<Eigen::Infinity>() <= config_.position_tolerance;
  }

  bool JointTargetBehaviour::hasValidConfig() const
  {
    return !config_.joint_names.empty() && !config_.joint_names.front().empty() &&
           !config_.targets.empty() && config_.gain > 0.0 && config_.position_tolerance >= 0.0;
  }

  const JointTarget *JointTargetBehaviour::targetByName(const std::string &target_name) const
  {
    const auto target = std::find_if(
        config_.targets.begin(), config_.targets.end(),
        [&target_name](const JointTarget &candidate) { return candidate.name == target_name; });

    if (target == config_.targets.end())
    {
      return nullptr;
    }

    return &*target;
  }

  const JointTarget *JointTargetBehaviour::activeTarget() const
  {
    return targetByName(active_target_name_);
  }

  std::optional<Eigen::VectorXd> JointTargetBehaviour::targetPositions(
      const JointTarget &target) const
  {
    if (target.positions.size() != config_.joint_names.size())
    {
      return std::nullopt;
    }

    Eigen::VectorXd positions(target.positions.size());
    for (std::size_t i = 0; i < target.positions.size(); ++i)
    {
      positions(static_cast<Eigen::Index>(i)) = target.positions[i];
    }

    return positions;
  }

  std::optional<Eigen::VectorXd> JointTargetBehaviour::orderedCurrentPositions(
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
