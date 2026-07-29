#pragma once

#include <string>
#include <vector>

#include "cartesian_command_manager/cartesian_command_manager_parameters.hpp"
#include "cartesian_command_manager/core/command_pipeline.hpp"
#include "cartesian_command_manager/core/types.hpp"

namespace cartesian_command_manager
{
  struct TopicConfig
  {
    std::string joystick_command;
    std::string geometric_state;
    std::string behaviour_state;
    std::string output_command;
    std::string ee_pose;
    std::string ee_vel;
    std::string ee_jac;
    std::string joint_states;
  };

  struct FrameConfig
  {
    std::string output_frame_id;
    std::string tip_frame_id;
    std::string default_input_frame_id;
  };

  struct InputConfig
  {
    manager_core::InputSource source{manager_core::InputSource::JOYSTICK};
    double timeout_sec{0.2};
    bool enabled{true};
  };

  struct ManagerConfig
  {
    double update_rate_hz{100.0};
    TopicConfig topics;
    FrameConfig frames;
    std::vector<InputConfig> inputs;
    manager_core::CommandPipelineConfig pipeline;
  };

  struct ParameterParseResult
  {
    ManagerConfig config;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;

    bool valid() const;
  };

  std::string normalizeParameterName(std::string name);
  ParameterParseResult parseManagerConfig(const Params &params);
} // namespace cartesian_command_manager
