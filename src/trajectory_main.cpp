#include "trajectory_pkg/trajectory_node.hpp"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
