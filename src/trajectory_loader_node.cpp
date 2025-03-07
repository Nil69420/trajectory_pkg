#include "trajectory_pkg/trajectory_loader_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <future>
#include <thread>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <memory>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace trajectory_loader {

//————————————————————————————————————————————
// MemoryMappedFile: Uses mmap (with RAII) for file reading
//————————————————————————————————————————————
class MemoryMappedFile {
public:
    MemoryMappedFile(const std::string &file_path) : data_(nullptr), size_(0) {
        fd_ = open(file_path.c_str(), O_RDONLY);
        if (fd_ == -1) {
            throw std::runtime_error("Failed to open file: " + file_path);
        }
        struct stat sb;
        if (fstat(fd_, &sb) == -1) {
            close(fd_);
            throw std::runtime_error("Failed to get file stats: " + file_path);
        }
        size_ = sb.st_size;
        data_ = static_cast<const char*>(mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0));
        if (data_ == MAP_FAILED) {
            close(fd_);
            throw std::runtime_error("Failed to mmap file: " + file_path);
        }
    }

    ~MemoryMappedFile() {
        if (data_ && data_ != MAP_FAILED) {
            munmap(const_cast<char*>(data_), size_);
        }
        if (fd_ != -1) {
            close(fd_);
        }
    }

    const char* data() const { return data_; }
    size_t size() const { return size_; }

private:
    int fd_;
    const char* data_;
    size_t size_;
};

//————————————————————————————————————————————
// Parser interface and implementations
//————————————————————————————————————————————
class AbstractTrajectoryParser {
public:
    virtual ~AbstractTrajectoryParser() = default;
    virtual std::vector<geometry_msgs::msg::Point> parse(const char* data, size_t size) = 0;
};

class CSVTrajectoryParser : public AbstractTrajectoryParser {
public:
    std::vector<geometry_msgs::msg::Point> parse(const char* data, size_t size) override {
        std::vector<geometry_msgs::msg::Point> points;
        std::istringstream iss(std::string(data, size));
        std::string line;
        // Skip header line
        if (std::getline(iss, line)) {
            while (std::getline(iss, line)) {
                std::stringstream ss(line);
                std::string time_str, x_str, y_str, z_str;
                geometry_msgs::msg::Point point;
                if (!std::getline(ss, time_str, ',')) continue;
                if (!std::getline(ss, x_str, ',')) continue;
                if (!std::getline(ss, y_str, ',')) continue;
                if (!std::getline(ss, z_str, ',')) continue;
                try {
                    point.x = std::stod(x_str);
                    point.y = std::stod(y_str);
                    point.z = std::stod(z_str);
                    points.push_back(point);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("CSVTrajectoryParser"),
                                 "Invalid number format in CSV: %s", line.c_str());
                }
            }
        }
        return points;
    }
};

class JSONTrajectoryParser : public AbstractTrajectoryParser {
public:
    std::vector<geometry_msgs::msg::Point> parse(const char* data, size_t size) override {
        std::vector<geometry_msgs::msg::Point> points;
        try {
            auto json_data = json::parse(std::string(data, size));
            for (const auto &point : json_data) {
                geometry_msgs::msg::Point p;
                p.x = point["x"].get<double>();
                p.y = point["y"].get<double>();
                p.z = point["z"].get<double>();
                points.push_back(p);
            }
        } catch (const json::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("JSONTrajectoryParser"),
                         "JSON parsing error: %s", e.what());
        }
        return points;
    }
};

class YAMLTraitoryParser : public AbstractTrajectoryParser {
public:
    std::vector<geometry_msgs::msg::Point> parse(const char* data, size_t size) override {
        std::vector<geometry_msgs::msg::Point> points;
        try {
            YAML::Node node = YAML::Load(std::string(data, size));
            auto trajectory = node["trajectory"];
            for (const auto &point : trajectory) {
                geometry_msgs::msg::Point p;
                p.x = point["x"].as<double>();
                p.y = point["y"].as<double>();
                p.z = point["z"].as<double>();
                points.push_back(p);
            }
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("YAMLTraitoryParser"),
                         "YAML parsing error: %s", e.what());
        }
        return points;
    }
};

//————————————————————————————————————————————
// TrajectoryLoaderNode Implementation
//————————————————————————————————————————————
TrajectoryLoaderNode::TrajectoryLoaderNode() : Node("trajectory_loader_node") {
    // Parameter declarations
    declare_parameter("file_path", "");
    declare_parameter("format", "csv");
    declare_parameter("source_frame", "map");

    // Get parameters
    file_path_ = get_parameter("file_path").as_string();
    format_ = get_parameter("format").as_string();
    source_frame_ = get_parameter("source_frame").as_string();

    // Initialize tf2 components
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("loaded_trajectory", 10);

    // Asynchronously load trajectory data using memory mapping and the appropriate parser
    load_trajectory();

    // Setup periodic publishing timer
    timer_ = create_wall_timer(500ms, [this]() { transform_and_publish(); });
}

TrajectoryLoaderNode::~TrajectoryLoaderNode() {
    // Clean-up if needed (RAII handles most resources)
}

void TrajectoryLoaderNode::load_trajectory() {
    auto future_points = std::async(std::launch::async, [this]() -> std::vector<geometry_msgs::msg::Point> {
        try {
            MemoryMappedFile mm_file(file_path_);
            std::unique_ptr<AbstractTrajectoryParser> parser;
            if (format_ == "csv") {
                parser = std::make_unique<CSVTrajectoryParser>();
            } else if (format_ == "json") {
                parser = std::make_unique<JSONTrajectoryParser>();
            } else if (format_ == "yaml") {
                parser = std::make_unique<YAMLTraitoryParser>();
            } else {
                RCLCPP_ERROR(get_logger(), "Unsupported format: %s", format_.c_str());
                return {};
            }
            return parser->parse(mm_file.data(), mm_file.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error loading trajectory: %s", e.what());
            return std::vector<geometry_msgs::msg::Point>{};
        }
    });
    trajectory_points_ = future_points.get();
    RCLCPP_INFO(get_logger(), "Loaded %zu trajectory points.", trajectory_points_.size());
}

bool TrajectoryLoaderNode::transform_point(const geometry_msgs::msg::Point& input,
                                             geometry_msgs::msg::Point& output) {
    try {
        geometry_msgs::msg::PoseStamped input_pose;
        input_pose.header.stamp = now();
        input_pose.header.frame_id = source_frame_;
        input_pose.pose.position = input;
        input_pose.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped output_pose;
        {  // Lock around the tf2 transform call in case of thread-safety concerns.
            std::lock_guard<std::mutex> lock(tf_mutex_);
            output_pose = tf_buffer_->transform(input_pose, "odom", tf2::durationFromSec(0.1));
        }
        output = output_pose.pose.position;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
        return false;
    }
}

void TrajectoryLoaderNode::transform_and_publish() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = now();
    marker.ns = "loaded_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Reserve space for performance
    marker.points.reserve(trajectory_points_.size());

    // Launch asynchronous transformations for each point
    std::vector<std::future<std::pair<bool, geometry_msgs::msg::Point>>> futures;
    for (const auto &point : trajectory_points_) {
        futures.push_back(std::async(std::launch::async, [this, point]() {
            geometry_msgs::msg::Point transformed;
            bool success = transform_point(point, transformed);
            return std::make_pair(success, transformed);
        }));
    }
    for (auto &fut : futures) {
        auto result = fut.get();
        if (result.first) {
            marker.points.push_back(result.second);
        }
    }

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

} // namespace trajectory_loader

// Register as a component for composition in ROS2
// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_loader::TrajectoryLoaderNode)
