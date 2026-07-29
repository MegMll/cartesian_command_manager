#include "cartesian_command_manager/ros/parameter_parsing.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>

namespace cartesian_command_manager
{
  namespace
  {
    constexpr double kDefaultUpdateRateHz = 100.0;
    constexpr double kDefaultInputTimeoutSec = 0.2;

    bool isFinite(double value)
    {
      return std::isfinite(value);
    }

    std::string quote(const std::string &value)
    {
      return "'" + value + "'";
    }

    double positiveOrDefault(double value, double default_value, const std::string &name,
                             std::vector<std::string> &warnings)
    {
      if (isFinite(value) && value > 0.0)
      {
        return value;
      }

      std::ostringstream message;
      message << name << " must be finite and > 0.0, got " << value << "; using " << default_value;
      warnings.push_back(message.str());
      return default_value;
    }

    double nonNegativeOrDefault(double value, double default_value, const std::string &name,
                                std::vector<std::string> &warnings)
    {
      if (isFinite(value) && value >= 0.0)
      {
        return value;
      }

      std::ostringstream message;
      message << name << " must be finite and >= 0.0, got " << value << "; using " << default_value;
      warnings.push_back(message.str());
      return default_value;
    }

    bool hasEmptyString(const std::vector<std::string> &values)
    {
      return std::any_of(values.begin(), values.end(),
                         [](const std::string &value) { return value.empty(); });
    }

    bool hasDuplicate(const std::vector<std::string> &values)
    {
      std::unordered_set<std::string> seen;
      for (const auto &value : values)
      {
        if (!seen.insert(value).second)
        {
          return true;
        }
      }

      return false;
    }

    void validateTopic(const std::string &value, const std::string &name,
                       std::vector<std::string> &errors)
    {
      if (value.empty())
      {
        errors.push_back(name + " must not be empty");
      }
    }

    std::vector<manager_core::JointTarget> parseJointTargets(const Params &params,
                                                             std::size_t joint_count,
                                                             std::vector<std::string> &warnings)
    {
      const auto &target_names = params.behaviours.joint_targets.target_names;
      const auto &positions = params.behaviours.joint_targets.positions;
      std::vector<manager_core::JointTarget> targets;

      if (target_names.empty())
      {
        warnings.push_back("behaviours.joint_targets.target_names is empty; joint-target "
                           "behaviour is disabled");
        return targets;
      }

      if (joint_count == 0)
      {
        warnings.push_back("behaviours.joint_targets.joint_names is empty; joint-target "
                           "behaviour is disabled");
        return targets;
      }

      if (positions.size() != target_names.size() * joint_count)
      {
        std::ostringstream message;
        message << "behaviours.joint_targets.positions has " << positions.size()
                << " values, expected " << target_names.size() * joint_count << " for "
                << target_names.size() << " targets and " << joint_count
                << " joints; incomplete joint targets are ignored";
        warnings.push_back(message.str());
      }

      targets.reserve(target_names.size());
      std::unordered_set<std::string> seen_targets;
      for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index)
      {
        auto target_name = normalizeParameterName(target_names[target_index]);
        if (target_name.empty())
        {
          warnings.push_back("Ignoring joint target with empty name at index " +
                             std::to_string(target_index));
          continue;
        }

        if (!seen_targets.insert(target_name).second)
        {
          warnings.push_back("Ignoring duplicate joint target " + quote(target_name));
          continue;
        }

        const auto first_position = target_index * joint_count;
        if (first_position + joint_count > positions.size())
        {
          warnings.push_back("Ignoring joint target " + quote(target_name) +
                             " because its position block is incomplete");
          continue;
        }

        const auto first = positions.begin() + static_cast<std::ptrdiff_t>(first_position);
        const auto last = first + static_cast<std::ptrdiff_t>(joint_count);
        if (!std::all_of(first, last, isFinite))
        {
          warnings.push_back("Ignoring joint target " + quote(target_name) +
                             " because at least one position is not finite");
          continue;
        }

        manager_core::JointTarget target;
        target.name = std::move(target_name);
        target.positions.assign(first, last);
        targets.push_back(std::move(target));
      }

      if (targets.empty())
      {
        warnings.push_back("No valid joint targets were configured; joint-target behaviour is "
                           "disabled");
      }

      return targets;
    }
  } // namespace

  bool ParameterParseResult::valid() const
  {
    return errors.empty();
  }

  std::string normalizeParameterName(std::string name)
  {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::replace(name.begin(), name.end(), '-', '_');
    return name;
  }

  ParameterParseResult parseManagerConfig(const Params &params)
  {
    ParameterParseResult result;
    auto &config = result.config;
    auto &warnings = result.warnings;
    auto &errors = result.errors;

    config.update_rate_hz =
        positiveOrDefault(params.update_rate_hz, kDefaultUpdateRateHz, "update_rate_hz", warnings);

    config.topics.joystick_command = params.topics.joystick_command;
    config.topics.geometric_state = params.topics.geometric_state;
    config.topics.behaviour_state = params.topics.behaviour_state;
    config.topics.ee_pose = params.topics.ee_pose;
    config.topics.ee_vel = params.topics.ee_vel;
    config.topics.ee_jac = params.topics.ee_jac;
    config.topics.joint_states = params.topics.joint_states;
    config.topics.output_command = params.topics.output_command;

    validateTopic(config.topics.joystick_command, "topics.joystick_command", errors);
    validateTopic(config.topics.geometric_state, "topics.geometric_state", errors);
    validateTopic(config.topics.behaviour_state, "topics.behaviour_state", errors);
    validateTopic(config.topics.ee_pose, "topics.ee_pose", errors);
    validateTopic(config.topics.ee_vel, "topics.ee_vel", errors);
    validateTopic(config.topics.ee_jac, "topics.ee_jac", errors);
    validateTopic(config.topics.joint_states, "topics.joint_states", errors);
    validateTopic(config.topics.output_command, "topics.output_command", errors);

    config.frames.output_frame_id = params.output_frame_id;
    config.frames.tip_frame_id = params.tip_frame_id;
    config.frames.default_input_frame_id = params.default_input_frame_id;

    config.inputs.push_back(
        InputConfig{manager_core::InputSource::JOYSTICK,
                    positiveOrDefault(params.inputs.joystick.timeout_sec, kDefaultInputTimeoutSec,
                                      "inputs.joystick.timeout_sec", warnings),
                    params.inputs.joystick.enabled});
    config.inputs.push_back(InputConfig{
        manager_core::InputSource::VISUAL_SERVOING,
        positiveOrDefault(params.inputs.visual_servoing.timeout_sec, kDefaultInputTimeoutSec,
                          "inputs.visual_servoing.timeout_sec", warnings),
        params.inputs.visual_servoing.enabled});

    config.pipeline.jaco.min_radius =
        nonNegativeOrDefault(params.shapers.jaco.min_radius, config.pipeline.jaco.min_radius,
                             "shapers.jaco.min_radius", warnings);
    config.pipeline.jaco.max_angular_velocity = nonNegativeOrDefault(
        params.shapers.jaco.max_angular_velocity, config.pipeline.jaco.max_angular_velocity,
        "shapers.jaco.max_angular_velocity", warnings);
    config.pipeline.snake.gain = params.shapers.snake.gain;
    if (!isFinite(config.pipeline.snake.gain))
    {
      warnings.push_back("shapers.snake.gain must be finite; using default");
      config.pipeline.snake.gain = manager_core::SnakeShaperConfig{}.gain;
    }

    auto joint_names = params.behaviours.joint_targets.joint_names;
    joint_names.erase(std::remove(joint_names.begin(), joint_names.end(), ""), joint_names.end());
    if (hasEmptyString(params.behaviours.joint_targets.joint_names))
    {
      warnings.push_back("behaviours.joint_targets.joint_names contains empty entries; they are "
                         "ignored");
    }
    if (hasDuplicate(joint_names))
    {
      warnings.push_back("behaviours.joint_targets.joint_names contains duplicate names; "
                         "joint-target behaviour may reject requests");
    }

    config.pipeline.joint_targets.joint_names = std::move(joint_names);
    config.pipeline.joint_targets.targets =
        parseJointTargets(params, config.pipeline.joint_targets.joint_names.size(), warnings);
    config.pipeline.joint_targets.gain =
        positiveOrDefault(params.behaviours.joint_targets.gain, config.pipeline.joint_targets.gain,
                          "behaviours.joint_targets.gain", warnings);
    config.pipeline.joint_targets.max_joint_velocity =
        nonNegativeOrDefault(params.behaviours.joint_targets.max_joint_velocity,
                             config.pipeline.joint_targets.max_joint_velocity,
                             "behaviours.joint_targets.max_joint_velocity", warnings);
    config.pipeline.joint_targets.max_linear_velocity =
        nonNegativeOrDefault(params.behaviours.joint_targets.max_linear_velocity,
                             config.pipeline.joint_targets.max_linear_velocity,
                             "behaviours.joint_targets.max_linear_velocity", warnings);
    config.pipeline.joint_targets.max_angular_velocity =
        nonNegativeOrDefault(params.behaviours.joint_targets.max_angular_velocity,
                             config.pipeline.joint_targets.max_angular_velocity,
                             "behaviours.joint_targets.max_angular_velocity", warnings);
    config.pipeline.joint_targets.position_tolerance =
        nonNegativeOrDefault(params.behaviours.joint_targets.position_tolerance,
                             config.pipeline.joint_targets.position_tolerance,
                             "behaviours.joint_targets.position_tolerance", warnings);

    return result;
  }
} // namespace cartesian_command_manager
