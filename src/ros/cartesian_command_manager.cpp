#include "cartesian_command_manager/ros/cartesian_command_manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <utility>

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

      if (normalized == "rotation")
      {
        return manager_core::GeometricState::ROTATION;
      }
      if (normalized == "translation")
      {
        return manager_core::GeometricState::TRANSLATION;
      }
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

    std::optional<manager_core::BehaviourState> parseBehaviourState(const std::string &state)
    {
      const auto normalized = normalizeStateName(state);

      if (normalized == "passthrough")
      {
        return manager_core::BehaviourState::PASSTHROUGH;
      }
      if (normalized == "shared")
      {
        return manager_core::BehaviourState::SHARED;
      }

      return std::nullopt;
    }

    void fillTwistMessage(const manager_core::CartesianCommand &command,
                          geometry_msgs::msg::Twist &twist)
    {
      twist.linear.x = command.linear.x();
      twist.linear.y = command.linear.y();
      twist.linear.z = command.linear.z();
      twist.angular.x = command.angular.x();
      twist.angular.y = command.angular.y();
      twist.angular.z = command.angular.z();
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
    output_command_topic_ = params_.topics.output_command;

    manager_core::CommandPipelineConfig pipeline_config;
    pipeline_config.snake.gain = params_.shapers.snake.gain;

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
  }

  void CartesianCommandManager::joystickCommandCallback(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    manager_core::CartesianCommand command;
    command.linear = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    command.angular = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
    pipeline_.setInputCommand(manager_core::InputSource::JOYSTICK, command, now().seconds());
  }

  void CartesianCommandManager::geometricStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto geometric_state = parseGeometricState(msg->data);
    if (!geometric_state)
    {
      RCLCPP_WARN(get_logger(), "Ignoring unknown geometric state '%s'", msg->data.c_str());
      return;
    }

    const auto requested_state = *geometric_state;
    const auto next_state =
        requested_state == current_geometric_state_ ? manager_core::GeometricState::BOTH
                                                    : requested_state;

    if (!pipeline_.setGeometricState(next_state))
    {
      RCLCPP_WARN(get_logger(), "Rejected geometric state '%s'", msg->data.c_str());
      return;
    }

    current_geometric_state_ = next_state;
  }

  void CartesianCommandManager::behaviourStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto behaviour_state = parseBehaviourState(msg->data);
    if (!behaviour_state)
    {
      RCLCPP_WARN(get_logger(), "Ignoring unknown behaviour state '%s'", msg->data.c_str());
      return;
    }

    const auto requested_state = *behaviour_state;
    const auto next_state =
        requested_state == current_behaviour_state_ ? manager_core::BehaviourState::PASSTHROUGH
                                                    : requested_state;

    if (!pipeline_.setBehaviourState(next_state))
    {
      RCLCPP_WARN(get_logger(), "Rejected behaviour state '%s'", msg->data.c_str());
      return;
    }

    current_behaviour_state_ = next_state;
  }

  void CartesianCommandManager::eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    robot_context_.ee_pose.position = {msg->pose.position.x, msg->pose.position.y,
                                       msg->pose.position.z};
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

  void CartesianCommandManager::updateVelocity()
  {
    const auto command = pipeline_.update(now().seconds(), 1.0 / update_rate_hz_, robot_context_);

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    if (command)
    {
      fillTwistMessage(*command, msg.twist);
    }
    cmd_pub_->publish(msg);
  }
} // namespace cartesian_command_manager
