#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace manager_core
{

  enum class InputSource
  {
    JOYSTICK,
    VISUAL_SERVOING
  };

  struct CartesianCommand
  {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  };

  struct TimedCartesianCommand
  {
    CartesianCommand command;
    double stamp_sec = 0.0;
    bool received = false;
  };

  struct InputChannel
  {
    TimedCartesianCommand latest;
    double timeout{0.2};
    bool enabled{true};
    double weight{1.};
  };

  enum class BehaviourState
  {
    PASSTHROUGH,
    SHARED
  };

  enum class GeometricState
  {
    ROTATION,
    TRANSLATION,
    BOTH,
    JACO,
    SNAKE,
  };

  struct CartesianPose
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  };

  struct CartesianVelocity
  {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  };

  struct RobotContext
  {
    CartesianPose ee_pose;
    CartesianVelocity ee_vel;
    Eigen::MatrixXd ee_jac;
  };
} // namespace manager_core
