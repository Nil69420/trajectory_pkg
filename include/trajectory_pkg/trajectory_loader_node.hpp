#ifndef TRAJECTORY_LOADER_NODE_HPP
#define TRAJECTORY_LOADER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <string>
#include <mutex>

namespace trajectory_loader {

class TrajectoryLoaderNode : public rclcpp::Node {
public:
    TrajectoryLoaderNode();
    ~TrajectoryLoaderNode();

private:
    void load_trajectory();
    void transform_and_publish();
    bool transform_point(const geometry_msgs::msg::Point& input, geometry_msgs::msg::Point& output);

    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Trajectory data
    std::vector<geometry_msgs::msg::Point> trajectory_points_;

    // Parameters
    std::string file_path_;
    std::string format_;
    std::string source_frame_;

    // Mutex for protecting tf2 transformations
    std::mutex tf_mutex_;
};

} // namespace trajectory_loader

#endif // TRAJECTORY_LOADER_NODE_HPP
