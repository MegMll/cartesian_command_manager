#include "cartesian_command_manager/ros/cartesian_command_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cartesian_command_manager::CartesianCommandManager>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}