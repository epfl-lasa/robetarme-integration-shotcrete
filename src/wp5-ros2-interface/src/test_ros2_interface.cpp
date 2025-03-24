#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "RosInterfaceHumble.hpp"

int main(int argc, char **argv) {
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  try {
    // Create a node to test RosInterfaceHumble
    auto testNode = std::make_shared<RosInterfaceHumble>("ur5_robot");

    RCLCPP_INFO(testNode->get_logger(), "Test node successfully created.");

    // Spin the node briefly to ensure no runtime errors
    rclcpp::spin_some(testNode);

    RCLCPP_INFO(testNode->get_logger(), "RosInterfaceHumble tested successfully.");
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Shutdown the ROS2 system
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
