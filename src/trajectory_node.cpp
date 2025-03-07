#include "trajectory_pkg/trajectory_node.hpp"
#include "trajectory_pkg/formatter.hpp"


using namespace std::chrono_literals;

// Constructor
TrajectoryNode::TrajectoryNode() : Node("trajectory_node") {
    declare_parameter("max_duration", 3600.0);
    max_duration_ = get_parameter("max_duration").as_double();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

    timer_ = create_wall_timer(100ms, [this]() { publishMarker(); });
    save_service_ = create_service<trajectory_pkg::srv::SaveTrajectory>(
        "save_trajectory", [this](const std::shared_ptr<rmw_request_id_t> req,
                                  const std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Request> request,
                                  std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Response> response) {
            saveTrajectoryCallback(req, request, response);
        });

    initializeMarkers();
}

// Initialize visualization markers
void TrajectoryNode::initializeMarkers() {
    marker_front_.header.frame_id = marker_back_.header.frame_id = "odom";
    marker_front_.ns = marker_back_.ns = "trajectory";
    marker_front_.id = marker_back_.id = 0;
    marker_front_.type = marker_back_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_front_.action = marker_back_.action = visualization_msgs::msg::Marker::ADD;
    marker_front_.scale.x = marker_back_.scale.x = 0.05;
    marker_front_.color.r = marker_back_.color.r = 1.0;
    marker_front_.color.a = marker_back_.color.a = 1.0;
}

// Odometry callback
void TrajectoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double squared_speed = 
        msg->twist.twist.linear.x * msg->twist.twist.linear.x +
        msg->twist.twist.linear.y * msg->twist.twist.linear.y +
        msg->twist.twist.linear.z * msg->twist.twist.linear.z;

    if (squared_speed < 0.0001) return;

    TrajectoryPoint tp{this->now(), msg->pose.pose.position};

    {
        std::unique_lock lock(trajectory_mutex_);
        trajectory_data_.emplace_back(tp);
        pruneOldData();
    }

    {
        std::lock_guard lock(marker_mutex_);
        marker_front_.points.emplace_back(tp.point);
    }
}

// Prune old trajectory data
void TrajectoryNode::pruneOldData() {
    const auto cutoff = this->now() - rclcpp::Duration::from_seconds(max_duration_);
    auto it = std::lower_bound(trajectory_data_.begin(), trajectory_data_.end(), cutoff,
        [](const TrajectoryPoint& tp, const rclcpp::Time& t) { return tp.stamp < t; });
    trajectory_data_.erase(trajectory_data_.begin(), it);
}

// Publish visualization markers
void TrajectoryNode::publishMarker() {
    visualization_msgs::msg::MarkerArray marker_array;
    {
        std::lock_guard lock(marker_mutex_);
        std::swap(marker_front_.points, marker_back_.points);
        marker_back_.header.stamp = this->now();
    }
    marker_array.markers.push_back(marker_back_);
    marker_pub_->publish(marker_array);
}

// Save trajectory callback
void TrajectoryNode::saveTrajectoryCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Request> request,
    std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Response> response) {
    
    std::shared_lock lock(trajectory_mutex_);
    const auto now = this->now();
    const auto lower_bound = now - rclcpp::Duration::from_seconds(request->duration);

    auto it = std::lower_bound(trajectory_data_.begin(), trajectory_data_.end(), lower_bound,
        [](const TrajectoryPoint& tp, const rclcpp::Time& t) { return tp.stamp < t; });

    const std::vector<TrajectoryPoint> filtered(it, trajectory_data_.end());
    lock.unlock();

    if (filtered.empty()) {
        response->success = false;
        response->message = "No data in time window";
        return;
    }

    try {
        if (request->format == "csv") {
            writeData<FormatTraits::CSV>(filtered, lower_bound, request->file_path);
        } else if (request->format == "json") {
            writeData<FormatTraits::JSON>(filtered, lower_bound, request->file_path);
        } else if (request->format == "yaml") {
            writeData<FormatTraits::YAML>(filtered, lower_bound, request->file_path);
        } else {
            throw std::runtime_error("Unsupported format");
        }
        
        response->success = true;
        response->message = "Saved to " + request->file_path;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}