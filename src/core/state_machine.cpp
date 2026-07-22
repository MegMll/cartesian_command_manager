#include "cartesian_command_manager/core/state_machine.hpp"

namespace manager_core
{
  bool StateMachine::setState(BehaviourState new_state)
  {
    if (!isCombinationAllowed(new_state, geometric_state_))
    {
      return false;
    }

    behaviour_state_ = new_state;
    return true;
  }

  bool StateMachine::setState(GeometricState new_state)
  {
    if (!isCombinationAllowed(behaviour_state_, new_state))
    {
      return false;
    }

    geometric_state_ = new_state;
    return true;
  }

  bool StateMachine::isCombinationAllowed(BehaviourState /*bState*/,
                                          GeometricState /*gState*/) const
  {
    return true;
  }
} // namespace manager_core
