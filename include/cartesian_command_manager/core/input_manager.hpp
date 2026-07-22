#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "cartesian_command_manager/core/types.hpp"

namespace manager_core
{
  class InputManager
  {
  public:
    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);

    void enableInputChannel(InputSource source);
    void disableInputChannel(InputSource source);

    bool isInputChannelEnabled(InputSource source) const;
    bool hasInputChannel(InputSource source) const;

    void setCommand(InputSource source, const CartesianCommand &command, double stamp_sec);
    bool hasValidcommand(InputSource source, double now_sec) const;
    std::optional<CartesianCommand> getCommand(InputSource source, double now_sec) const;

    std::vector<InputSource> getValidSources(double now_sec) const;

    void clearCommand(InputSource source);
    void clearAllCommands();

    std::optional<CartesianCommand> getFullCommand(double now_sec) const;

  private:
    std::unordered_map<InputSource, InputChannel> inputs_;
  };
} // namespace manager_core
