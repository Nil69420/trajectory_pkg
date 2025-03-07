#ifndef FORMATTER_HPP
#define FORMATTER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include "trajectory_pkg/trajectory_node.hpp"  

// Namespace for format-specific traits
namespace FormatTraits {
    struct CSV {};
    struct JSON {};
    struct YAML {};

    // Format detection
    template<typename T> struct FormatTag {};
    template<> struct FormatTag<CSV> { static constexpr const char* str = "csv"; };
    template<> struct FormatTag<JSON> { static constexpr const char* str = "json"; };
    template<> struct FormatTag<YAML> { static constexpr const char* str = "yaml"; };
}

// --- Formatter Classes ---

// Generic formatter template declaration
template<typename Format>
class Formatter {
public:
    static void serializeHeader(std::string& buffer);
    static void serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p);
    static void serializeFooter(std::string& buffer);
};

// Specialization for CSV format
template<>
class Formatter<FormatTraits::CSV> {
public:
    static constexpr size_t ESTIMATED_LINE_LENGTH = 256;
    static void serializeHeader(std::string& buffer);
    static void serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p);
    static void serializeFooter(std::string& buffer);
};

// Specialization for JSON format
template<>
class Formatter<FormatTraits::JSON> {
public:
    static void serializeHeader(std::string& buffer);
    static void serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p);
    static void serializeFooter(std::string& buffer);
};

// Specialization for YAML format
template<>
class Formatter<FormatTraits::YAML> {
public:
    static void serializeHeader(std::string& buffer);
    static void serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p);
    static void serializeFooter(std::string& buffer);
};

// --- Writer Function Template ---
template<typename Format>
void writeData(const std::vector<TrajectoryPoint>& data,
               const rclcpp::Time& lower_bound,
               const std::string& path) {
    std::string buffer;
    Formatter<Format>::serializeHeader(buffer);

    for (size_t i = 0; i < data.size(); ++i) {
        const auto& tp = data[i];
        const double t = (tp.stamp - lower_bound).seconds();
        Formatter<Format>::serializePoint(buffer, t, tp.point);

        if constexpr (std::is_same_v<Format, FormatTraits::JSON>) {
            if (i != data.size() - 1) {
                buffer.append(",\n");
            }
        }
    }

    Formatter<Format>::serializeFooter(buffer);

    std::ofstream file(path);
    file << buffer;
}

// Declaration of the CSV specialization of writeData.
template<>
void writeData<FormatTraits::CSV>(const std::vector<TrajectoryPoint>& data,
                                  const rclcpp::Time& lower_bound,
                                  const std::string& path);

#endif // FORMATTER_HPP
