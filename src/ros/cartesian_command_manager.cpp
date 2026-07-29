#include "cartesian_command_manager/ros/cartesian_command_manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cartesian_command_manager
{
  namespace
  {
    std::string normalizeStateName(std::string state)
    {
      std::transform(state.begin(), state.end(), state.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      std::replace(state.begin(), state.end(), '-', '_');
      return state;
    }

    std::optional<manager_core::GeometricState> parseGeometricState(const std::string &state)
    {
      const auto normalized = normalizeStateName(state);

      if (normalized == "both")
      {
        return manager_core::GeometricState::BOTH;
      }
      if (normalized == "jaco")
      {
        return manager_core::GeometricState::JACO;
      }
      if (normalized == "snake")
      {
        return manager_core::GeometricState::SNAKE;
      }

      return std::nullopt;
    }

    struct BehaviourRequest
    {
      manager_core::BehaviourState state{manager_core::BehaviourState::PASSTHROUGH};
      std::string joint_target_name;
    };

    std::optional<BehaviourRequest> parseBehaviourState(const std::string &state)
    {
      const auto normalized = normalizeStateName(state);

      if (normalized == "passthrough")
      {
        return BehaviourRequest{manager_core::BehaviourState::PASSTHROUGH, ""};
      }
      if (normalized == "homing" || normalized == "go_home")
      {
        return BehaviourRequest{manager_core::BehaviourState::JOINT_TARGET, "home"};
      }
      if (normalized.rfind("go_", 0) == 0 && normalized.size() > 3)
      {
        return BehaviourRequest{manager_core::BehaviourState::JOINT_TARGET, normalized.substr(3)};
      }

      return std::nullopt;
    }

    void fillTwistMessage(const manager_core::CartesianCommand &command,
                          geometry_msgs::msg::TwistStamped &msg)
    {
      msg.header.frame_id = command.frame_id;
      msg.twist.linear.x = command.linear.x();
      msg.twist.linear.y = command.linear.y();
      msg.twist.linear.z = command.linear.z();
      msg.twist.angular.x = command.angular.x();
      msg.twist.angular.y = command.angular.y();
      msg.twist.angular.z = command.angular.z();
    }

    std::vector<manager_core::JointTarget> buildJointTargets(
        const std::vector<std::string> &target_names, const std::vector<double> &positions,
        std::size_t joint_count)
    {
      std::vector<manager_core::JointTarget> targets;
      targets.reserve(target_names.size());

      if (joint_count == 0)
      {
        return targets;
      }

      for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index)
      {
        const auto first_position = target_index * joint_count;
        if (first_position + joint_count > positions.size())
        {
          break;
        }

        manager_core::JointTarget target;
        target.name = normalizeStateName(target_names[target_index]);
        target.positions.assign(positions.begin() + static_cast<std::ptrdiff_t>(first_position),
                                positions.begin() +
                                    static_cast<std::ptrdiff_t>(first_position + joint_count));
        targets.push_back(std::move(target));
      }

      return targets;
    }

  } // namespace

  CartesianCommandManager::CartesianCommandManager(const rclcpp::NodeOptions &options)
      : rclcpp::Node("cartesian_command_manager", options)
  {
    param_listener_ = std::make_shared<cartesian_command_manager::ParamListener>(this);

    readParameters();
    setupSubscribers();
    setupPublishers();

    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&CartesianCommandManager::updateVelocity, this));

    RCLCPP_INFO(get_logger(), "Cartesian command manager publishing '%s'",
                output_command_topic_.c_str());
  }

  void CartesianCommandManager::readParameters()
  {
    params_ = param_listener_->get_params();

    update_rate_hz_ = params_.update_rate_hz;
    if (update_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid update_rate_hz %.3f, using 100.0 Hz", update_rate_hz_);
      update_rate_hz_ = 100.0;
    }

    joystick_command_topic_ = params_.topics.joystick_command;
    geometric_state_topic_ = params_.topics.geometric_state;
    behaviour_state_topic_ = params_.topics.behaviour_state;
    ee_pose_topic_ = params_.topics.ee_pose;
    ee_vel_topic_ = params_.topics.ee_vel;
    ee_jac_topic_ = params_.topics.ee_jac;
    joint_states_topic_ = params_.topics.joint_states;
    output_command_topic_ = params_.topics.output_command;
    output_frame_id_ = params_.output_frame_id;
    tip_frame_id_ = params_.tip_frame_id;
    default_input_frame_id_ = params_.default_input_frame_id;

    manager_core::CommandPipelineConfig pipeline_config;
    pipeline_config.jaco.min_radius = params_.shapers.jaco.min_radius;
    pipeline_config.jaco.max_angular_velocity = params_.shapers.jaco.max_angular_velocity;
    pipeline_config.snake.gain = params_.shapers.snake.gain;
    pipeline_config.joint_targets.joint_names = params_.behaviours.joint_targets.joint_names;
    pipeline_config.joint_targets.targets =
        buildJointTargets(params_.behaviours.joint_targets.target_names,
                          params_.behaviours.joint_targets.positions,
                          pipeline_config.joint_targets.joint_names.size());
    pipeline_config.joint_targets.gain = params_.behaviours.joint_targets.gain;
    pipeline_config.joint_targets.max_joint_velocity =
        params_.behaviours.joint_targets.max_joint_velocity;
    pipeline_config.joint_targets.max_linear_velocity =
        params_.behaviours.joint_targets.max_linear_velocity;
    pipeline_config.joint_targets.max_angular_velocity =
        params_.behaviours.joint_targets.max_angular_velocity;
    pipeline_config.joint_targets.position_tolerance =
        params_.behaviours.joint_targets.position_tolerance;

    if (pipeline_config.joint_targets.joint_names.empty() ||
        pipeline_config.joint_targets.joint_names.front().empty())
    {
      RCLCPP_WARN(get_logger(),
                  "Joint-target behaviour has no configured joints; target requests will be "
                  "rejected");
    }
    else if (pipeline_config.joint_targets.targets.size() !=
             params_.behaviours.joint_targets.target_names.size())
    {
      RCLCPP_WARN(get_logger(),
                  "Joint-target behaviour has %zu target names but only %zu complete target "
                  "position blocks",
                  params_.behaviours.joint_targets.target_names.size(),
                  pipeline_config.joint_targets.targets.size());
    }
    else if (params_.behaviours.joint_targets.positions.size() !=
             params_.behaviours.joint_targets.target_names.size() *
                 pipeline_config.joint_targets.joint_names.size())
    {
      RCLCPP_WARN(get_logger(),
                  "Joint-target behaviour has %zu positions for %zu targets and %zu joints; extra "
                  "positions will be ignored",
                  params_.behaviours.joint_targets.positions.size(),
                  params_.behaviours.joint_targets.target_names.size(),
                  pipeline_config.joint_targets.joint_names.size());
    }

    pipeline_.configure(pipeline_config);
    pipeline_.setBehaviourState(current_behaviour_state_);
    pipeline_.setGeometricState(current_geometric_state_);
    pipeline_.addInputChannel(manager_core::InputSource::JOYSTICK,
                              params_.inputs.joystick.timeout_sec, params_.inputs.joystick.enabled);
  }

  void CartesianCommandManager::setupSubscribers()
  {
    joystick_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        joystick_command_topic_, 10,
        std::bind(&CartesianCommandManager::joystickCommandCallback, this, std::placeholders::_1));

    ee_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        ee_pose_topic_, 10,
        std::bind(&CartesianCommandManager::eePoseCallback, this, std::placeholders::_1));
    ee_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        ee_vel_topic_, 10,
        std::bind(&CartesianCommandManager::eeVelCallback, this, std::placeholders::_1));
    ee_jac_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        ee_jac_topic_, 10,
        std::bind(&CartesianCommandManager::eeJacCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        std::bind(&CartesianCommandManager::jointStateCallback, this, std::placeholders::_1));

    geometric_state_sub_ = create_subscription<std_msgs::msg::String>(
        geometric_state_topic_, 10,
        std::bind(&CartesianCommandManager::geometricStateCallback, this, std::placeholders::_1));
    behaviour_state_sub_ = create_subscription<std_msgs::msg::String>(
        behaviour_state_topic_, 10,
        std::bind(&CartesianCommandManager::behaviourStateCallback, this, std::placeholders::_1));
  }

  void CartesianCommandManager::setupPublishers()
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_command_topic_, 10);
    behaviour_state_pub_ = create_publisher<std_msgs::msg::String>(behaviour_state_topic_, 10);
  }

  void CartesianCommandManager::joystickCommandCallback(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    manager_core::CartesianCommand command;
    command.linear = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    command.angular = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
    command.frame_id = msg->header.frame_id;

    const auto transformed = transformCommandToOutputFrame(command, "joystick");
    if (!transformed)
    {
      return;
    }

    pipeline_.setInputCommand(manager_core::InputSource::JOYSTICK, *transformed, now().seconds());
  }

  void CartesianCommandManager::geometricStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto geometric_state = parseGeometricState(msg->data);
    if (!geometric_state)
    {
      RCLCPP_WARN(get_logger(), "Ignoring unknown geometric state '%s'", msg->data.c_str());
      return;
    }

    pipeline_.setGeometricState(*geometric_state);
    current_geometric_state_ = *geometric_state;
  }

  void CartesianCommandManager::behaviourStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto behaviour_state = parseBehaviourState(msg->data);
    if (!behaviour_state)
    {
      RCLCPP_WARN(get_logger(), "Ignoring unknown behaviour state '%s'", msg->data.c_str());
      return;
    }

    if (behaviour_state->state == manager_core::BehaviourState::JOINT_TARGET)
    {
      if (!pipeline_.setJointTarget(behaviour_state->joint_target_name))
      {
        RCLCPP_WARN(get_logger(), "Rejecting unknown joint target '%s'",
                    behaviour_state->joint_target_name.c_str());
        return;
      }

      const auto target_error = pipeline_.validateJointTarget(robot_context_);
      if (target_error)
      {
        RCLCPP_WARN(get_logger(), "Rejecting joint-target request '%s': %s",
                    behaviour_state->joint_target_name.c_str(), target_error->c_str());
        pipeline_.setBehaviourState(manager_core::BehaviourState::PASSTHROUGH);
        current_behaviour_state_ = manager_core::BehaviourState::PASSTHROUGH;
        publishBehaviourState("passthrough");
        return;
      }
    }

    pipeline_.setBehaviourState(behaviour_state->state);
    current_behaviour_state_ = behaviour_state->state;
  }

  void CartesianCommandManager::eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    robot_context_.ee_pose.position = {msg->pose.position.x, msg->pose.position.y,
                                       msg->pose.position.z};
    robot_context_.ee_pose.frame_id = msg->header.frame_id;
    robot_context_.ee_pose.orientation =
        Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);

    if (robot_context_.ee_pose.orientation.norm() > 0.0)
    {
      robot_context_.ee_pose.orientation.normalize();
    }
    else
    {
      robot_context_.ee_pose.orientation = Eigen::Quaterniond::Identity();
    }
  }

  void CartesianCommandManager::eeVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    robot_context_.ee_vel.frame_id = msg->header.frame_id;
    robot_context_.ee_vel.linear = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    robot_context_.ee_vel.angular = {msg->twist.angular.x, msg->twist.angular.y,
                                     msg->twist.angular.z};
  }

  void CartesianCommandManager::eeJacCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto data_offset = static_cast<std::size_t>(msg->layout.data_offset);
    if (data_offset > msg->data.size())
    {
      RCLCPP_WARN(get_logger(),
                  "Ignoring Jacobian message with data_offset %zu larger than data size %zu",
                  data_offset, msg->data.size());
      return;
    }

    std::size_t rows = 0;
    std::size_t cols = 0;
    std::size_t row_stride = 0;
    std::size_t col_stride = 1;

    if (msg->layout.dim.size() >= 2)
    {
      rows = static_cast<std::size_t>(msg->layout.dim[0].size);
      cols = static_cast<std::size_t>(msg->layout.dim[1].size);
      row_stride = static_cast<std::size_t>(msg->layout.dim[1].stride);
      col_stride = 1;

      if (row_stride == 0)
      {
        row_stride = cols;
      }
    }
    else
    {
      constexpr std::size_t kCartesianJacobianRows = 6;
      const auto available_data = msg->data.size() - data_offset;
      if (available_data % kCartesianJacobianRows != 0)
      {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring Jacobian message without 2D layout: data size %zu is not divisible by 6",
            available_data);
        return;
      }

      rows = kCartesianJacobianRows;
      cols = available_data / kCartesianJacobianRows;
      row_stride = cols;
    }

    if (rows == 0 || cols == 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring empty Jacobian message with shape %zux%zu", rows, cols);
      return;
    }

    const auto last_index = data_offset + (rows - 1) * row_stride + (cols - 1) * col_stride;
    if (last_index >= msg->data.size())
    {
      RCLCPP_WARN(get_logger(),
                  "Ignoring Jacobian message with shape %zux%zu and strides %zu/%zu: "
                  "requires index %zu but data size is %zu",
                  rows, cols, row_stride, col_stride, last_index, msg->data.size());
      return;
    }

    Eigen::MatrixXd jacobian(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
    for (std::size_t row = 0; row < rows; ++row)
    {
      for (std::size_t col = 0; col < cols; ++col)
      {
        jacobian(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
            msg->data[data_offset + row * row_stride + col * col_stride];
      }
    }

    robot_context_.ee_jac = std::move(jacobian);
  }

  void CartesianCommandManager::jointStateCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size())
    {
      RCLCPP_WARN(get_logger(), "Ignoring JointState message with %zu names and %zu positions",
                  msg->name.size(), msg->position.size());
      return;
    }

    robot_context_.joint_names = msg->name;
    robot_context_.joint_positions =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(msg->position.size()));

    for (std::size_t i = 0; i < msg->position.size(); ++i)
    {
      robot_context_.joint_positions(static_cast<Eigen::Index>(i)) = msg->position[i];
    }
  }

  void CartesianCommandManager::updateVelocity()
  {
    if (current_behaviour_state_ == manager_core::BehaviourState::JOINT_TARGET)
    {
      const auto target_error = pipeline_.validateJointTarget(robot_context_);
      if (target_error)
      {
        RCLCPP_WARN(get_logger(), "Aborting joint-target behaviour: %s", target_error->c_str());
        pipeline_.setBehaviourState(manager_core::BehaviourState::PASSTHROUGH);
        current_behaviour_state_ = manager_core::BehaviourState::PASSTHROUGH;
        publishBehaviourState("passthrough");
      }
    }

    const bool target_reached = current_behaviour_state_ == manager_core::BehaviourState::JOINT_TARGET &&
                                pipeline_.isJointTargetReached(robot_context_);

    const auto command = pipeline_.update(now().seconds(), 1.0 / update_rate_hz_, robot_context_);
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = output_frame_id_;
    if (command)
    {
      auto output = *command;
      if (output.frame_id.empty())
      {
        output.frame_id = output_frame_id_;
      }
      else if (output.frame_id != output_frame_id_)
      {
        const auto transformed = transformCommandToOutputFrame(output, "pipeline output");
        if (!transformed)
        {
          cmd_pub_->publish(msg);
          return;
        }
        output = *transformed;
      }
      fillTwistMessage(output, msg);
    }

    cmd_pub_->publish(msg);

    if (target_reached)
    {
      pipeline_.setBehaviourState(manager_core::BehaviourState::PASSTHROUGH);
      current_behaviour_state_ = manager_core::BehaviourState::PASSTHROUGH;
      publishBehaviourState("passthrough");
      RCLCPP_INFO(get_logger(), "Joint target reached, switching behaviour to passthrough");
    }
  }

  void CartesianCommandManager::publishBehaviourState(const std::string &state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    behaviour_state_pub_->publish(msg);
  }

  std::optional<manager_core::CartesianCommand>
  CartesianCommandManager::transformCommandToOutputFrame(manager_core::CartesianCommand command,
                                                        const std::string &source_name)
  {
    if (command.frame_id.empty())
    {
      command.frame_id = default_input_frame_id_;
    }

    if (output_frame_id_.empty() || command.frame_id == output_frame_id_)
    {
      command.frame_id = output_frame_id_;
      return command;
    }

    if (robot_context_.ee_pose.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Cannot transform %s command from '%s' to '%s': no end-effector pose "
                           "frame is available yet",
                           source_name.c_str(), command.frame_id.c_str(),
                           output_frame_id_.c_str());
      return std::nullopt;
    }

    const Eigen::Matrix3d output_R_tip = robot_context_.ee_pose.orientation.toRotationMatrix();
    if (command.frame_id == tip_frame_id_ && output_frame_id_ == robot_context_.ee_pose.frame_id)
    {
      command.linear = output_R_tip * command.linear;
      command.angular = output_R_tip * command.angular;
      command.frame_id = output_frame_id_;
      return command;
    }

    if (command.frame_id == robot_context_.ee_pose.frame_id && output_frame_id_ == tip_frame_id_)
    {
      command.linear = output_R_tip.transpose() * command.linear;
      command.angular = output_R_tip.transpose() * command.angular;
      command.frame_id = output_frame_id_;
      return command;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot transform %s command from '%s' to '%s': supported frames are "
                         "'%s' and '%s'",
                         source_name.c_str(), command.frame_id.c_str(), output_frame_id_.c_str(),
                         robot_context_.ee_pose.frame_id.c_str(), tip_frame_id_.c_str());
    return std::nullopt;
  }
} // namespace cartesian_command_manager
