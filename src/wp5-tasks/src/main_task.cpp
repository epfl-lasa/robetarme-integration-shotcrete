#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "TaskFSM.h"
#include "TaskFactory.h"

int main(int argc, char **argv) {
  // Initialize ROS 2 node
  rclcpp::init(argc, argv);

  double rosFreq = 125.0;
  TaskFactory taskFactory;
  // Create a temporary node to retrieve the parameter
  auto temp_node = std::make_shared<rclcpp::Node>("temp_node");

  // Declare and get the taskType parameter
  temp_node->declare_parameter<std::string>("taskType", "shotcreteKUL");
  std::string taskType;
  if (!temp_node->get_parameter("taskType", taskType)) {
    RCLCPP_ERROR(temp_node->get_logger(), "No taskType parameter received");
    return 1;
  }
  temp_node.reset(); // Destroy the temporary node

  // Construct the node name dynamically
  std::string node_name = taskType + "_wp5_main_task_node";
  auto node = rclcpp::Node::make_shared(node_name);

  RCLCPP_INFO(node->get_logger(), "Node started with name: %s", node_name.c_str());
  // Validate the taskType parameter
  std::vector<std::string> allowed_values = taskFactory.getTaskTypes();
  if (std::find(allowed_values.begin(), allowed_values.end(), taskType) == allowed_values.end()) {
    RCLCPP_ERROR(node->get_logger(), "Invalid taskType: %s", taskType.c_str());
    return 1;
  }

  // Load parameters from YAML file
  std::string alternativeYamlPath = std::string(WP5_TASKS_DIR) + "/config/tasks_config.yaml";
  std::string yamlPath = std::string(WP5_TASKS_DIR) + "/../../config/tasks_config.yaml";
  std::string nameYaml = "ros2_ws/config/tasks_config.yaml";

  // Check if the alternative YAML file exists
  std::ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    std::cout << "Using general YAML file: " << nameYaml << std::endl;
  } else {
    yamlPath = alternativeYamlPath;
    std::cout << "Using local YAML file: " << yamlPath << std::endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);
  YAML::Node taskTypeYaml = config[taskType];

  std::string robotName = taskTypeYaml["robot_name"].as<std::string>();

  // Create TaskFSM using taskFsm_ alias
  std::shared_ptr<ITaskBase> task = taskFactory.createTask(taskType, rosFreq, robotName);
  taskFsm_ internalFSM(task); // taskFsm_ is the state machine backend for TaskFSM

  // Start the state machine
  internalFSM.start();

  // Trigger a state machine event
  internalFSM.process_event(Start());

  RCLCPP_INFO(node->get_logger(), "State machine successfully started.");

  rclcpp::shutdown();
  return 0;
}
