#pragma once

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/string.hpp"

#include "cartesian_command_manager/cartesian_command_manager_parameters.hpp"
#include "cartesian_command_manager/core/command_pipeline.hpp"
#include "cartesian_command_manager/ros/parameter_parsing.hpp"

namespace cartesian_command_manager
{
  class CartesianCommandManager : public rclcpp::Node
  {
  public:
    explicit CartesianCommandManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void setupSubscribers();
    void setupPublishers();
    void readParameters();

    void joystickCommandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void geometricStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void behaviourStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void eeVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void eeJacCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void updateVelocity();
    void publishBehaviourState(const std::string &state);
    std::optional<manager_core::CartesianCommand> transformCommandToOutputFrame(
        manager_core::CartesianCommand command, const std::string &source_name);

    manager_core::CommandPipeline pipeline_;
    manager_core::RobotContext robot_context_;
    manager_core::GeometricState current_geometric_state_{manager_core::GeometricState::BOTH};
    manager_core::BehaviourState current_behaviour_state_{
        manager_core::BehaviourState::PASSTHROUGH};

    std::shared_ptr<cartesian_command_manager::ParamListener> param_listener_;
    ManagerConfig config_;

    double update_rate_hz_{100.0};

    std::string joystick_command_topic_;
    std::string geometric_state_topic_;
    std::string behaviour_state_topic_;
    std::string output_command_topic_;
    std::string ee_pose_topic_;
    std::string ee_vel_topic_;
    std::string ee_jac_topic_;
    std::string joint_states_topic_;
    std::string output_frame_id_;
    std::string tip_frame_id_;
    std::string default_input_frame_id_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joystick_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr geometric_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behaviour_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ee_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_jac_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behaviour_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace cartesian_command_manager
