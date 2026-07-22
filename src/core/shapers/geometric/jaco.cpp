#include "cartesian_command_manager/core/shapers/geometric/jaco.hpp"

namespace manager_core
{
  CartesianCommand JacoShaper::update(const CartesianCommand &input, const RobotContext &context,
                                      double /*dt_sec*/)
  {
    CartesianCommand output = input;

    const double x_E = context.ee_pose.position.x();
    const double y_E = context.ee_pose.position.y();
    const double v_x = input.linear.x();
    const double v_y = input.linear.y();
    const double denom = x_E * x_E + y_E * y_E;

    double omega_z_baseline = 0.0;
    if (denom > 0.0)
    {
      omega_z_baseline = (x_E * v_y - y_E * v_x) / denom;
    }

    output.angular = Eigen::Vector3d(0.0, 0.0, omega_z_baseline);
    return output;
  }

  void JacoShaper::reset()
  {
  }
} // namespace manager_core
