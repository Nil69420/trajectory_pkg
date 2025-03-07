#include "trajectory_pkg/trajectory_loader_node.hpp"


using trajectory_loader::TrajectoryLoaderNode;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryLoaderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
