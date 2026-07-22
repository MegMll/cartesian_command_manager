#pragma once

#include "cartesian_command_manager/core/shapers/shaper.hpp"

namespace manager_core
{
  class JacoShaper : public Shaper
  {
  public:
    JacoShaper() = default;

    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;
  };
} // namespace manager_core
