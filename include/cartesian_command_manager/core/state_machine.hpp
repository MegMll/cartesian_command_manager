#pragma once

#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  class StateMachine
  {
  public:
    BehaviourState behaviour_state() const
    {
      return behaviour_state_;
    }

    GeometricState geometric_state() const
    {
      return geometric_state_;
    }

    bool setState(BehaviourState new_state);
    bool setState(GeometricState new_state);

  private:
    bool isCombinationAllowed(BehaviourState bState, GeometricState gState) const;

    BehaviourState behaviour_state_{BehaviourState::PASSTHROUGH};
    GeometricState geometric_state_{GeometricState::BOTH};
  };
} // namespace manager_core
