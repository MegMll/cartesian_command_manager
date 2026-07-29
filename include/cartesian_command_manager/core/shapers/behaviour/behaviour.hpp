#pragma once

#include <optional>
#include <string>

#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  class Behaviour
  {
  public:
    virtual ~Behaviour() = default;

    virtual std::optional<CartesianCommand> update(const RobotContext &context) const = 0;
    virtual std::optional<std::string> validate(const RobotContext &context) const = 0;
    virtual bool isComplete(const RobotContext &context) const = 0;
    virtual void reset()
    {
    }
  };
} // namespace manager_core
