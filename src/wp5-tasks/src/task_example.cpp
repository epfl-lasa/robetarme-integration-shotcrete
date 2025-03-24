// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ITaskBase.h"

class TaskNode : public rclcpp::Node {
public:
  TaskNode() : Node("task_node") {
    RCLCPP_INFO(this->get_logger(), "TaskNode started!");
    ITaskBase_ = std::make_shared<ITaskBase>(150, "ur5_robot");
  }

private:
  std::shared_ptr<ITaskBase> ITaskBase_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
