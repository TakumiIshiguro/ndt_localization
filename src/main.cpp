#include "wheel_lidar_localization/localization_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<wheel_lidar_localization::LocalizationNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "wheel_lidar_localization: " << e.what() << std::endl;
    result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
