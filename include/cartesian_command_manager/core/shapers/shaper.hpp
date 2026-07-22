#pragma once

#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  class Shaper
  {
  public:
    virtual ~Shaper() = default;

    virtual CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                                    double dt_sec) = 0;

    virtual void reset() = 0;
  };
} // namespace manager_core
