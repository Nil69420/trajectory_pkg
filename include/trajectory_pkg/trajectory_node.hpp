#ifndef TRAJECTORY_NODE_HPP
#define TRAJECTORY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "trajectory_pkg/srv/save_trajectory.hpp"
#include <vector>
#include <shared_mutex>
#include <string>
#include <memory>
#include <fstream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Structure to hold trajectory points with timestamps
struct TrajectoryPoint {
    rclcpp::Time stamp;
    geometry_msgs::msg::Point point;
};

// Main ROS2 Node for trajectory tracking and visualization
class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode();

private:
    // Callback for odometry messages
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Timer callback to publish visualization markers
    void publishMarker();

    // Service callback to save trajectory data
    void saveTrajectoryCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Request> request,
        std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Response> response);

    // Helper function to initialize visualization markers
    void initializeMarkers();

    // Helper function to prune old trajectory data
    void pruneOldData();

    // Member variables
    std::shared_mutex trajectory_mutex_; // Mutex for thread-safe access to trajectory data
    std::vector<TrajectoryPoint> trajectory_data_; // Stores trajectory points
    double max_duration_; // Maximum duration to keep trajectory data

    std::mutex marker_mutex_; // Mutex for thread-safe access to markers
    visualization_msgs::msg::Marker marker_front_, marker_back_; // Double-buffered markers

    // ROS2 interfaces
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<trajectory_pkg::srv::SaveTrajectory>::SharedPtr save_service_;
};

#endif // TRAJECTORY_NODE_HPP