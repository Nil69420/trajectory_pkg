#include "trajectory_pkg/formatter.hpp"


// --- CSV Formatter Implementation ---

void Formatter<FormatTraits::CSV>::serializeHeader(std::string& buffer) {
    // Reserve memory.
    buffer.reserve(ESTIMATED_LINE_LENGTH * 1000);
    buffer += "time,x,y,z\n";
}

void Formatter<FormatTraits::CSV>::serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p) {
    char line[256];
    const int len = snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f\n", time, p.x, p.y, p.z);
    buffer.append(line, len);
}

void Formatter<FormatTraits::CSV>::serializeFooter(std::string& /*buffer*/) {
    //  parameter name omitted to suppress unused warnings.
}

// --- JSON Formatter Implementation ---

void Formatter<FormatTraits::JSON>::serializeHeader(std::string& buffer) {
    buffer = "[\n";
}

void Formatter<FormatTraits::JSON>::serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p) {
    buffer.append("  {\"time\":")
          .append(std::to_string(time))
          .append(",\"x\":")
          .append(std::to_string(p.x))
          .append(",\"y\":")
          .append(std::to_string(p.y))
          .append(",\"z\":")
          .append(std::to_string(p.z))
          .append("}");
}

void Formatter<FormatTraits::JSON>::serializeFooter(std::string& buffer) {
    buffer.append("\n]");
}

// --- YAML Formatter Implementation ---

void Formatter<FormatTraits::YAML>::serializeHeader(std::string& buffer) {
    buffer = "---\ntrajectory:\n";
}

void Formatter<FormatTraits::YAML>::serializePoint(std::string& buffer, double time, const geometry_msgs::msg::Point& p) {
    buffer.append("  - time: ")
          .append(std::to_string(time))
          .append("\n    x: ")
          .append(std::to_string(p.x))
          .append("\n    y: ")
          .append(std::to_string(p.y))
          .append("\n    z: ")
          .append(std::to_string(p.z))
          .append("\n");
}

void Formatter<FormatTraits::YAML>::serializeFooter(std::string& /*buffer*/) {
    
}

// --- CSV writeData Specialization Implementation ---

template<>
void writeData<FormatTraits::CSV>(const std::vector<TrajectoryPoint>& data,
                                  const rclcpp::Time& lower_bound,
                                  const std::string& path) {
    const int fd = open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (fd == -1) {
        throw std::runtime_error("Failed to open file");
    }

    std::string buffer;
    buffer.reserve(Formatter<FormatTraits::CSV>::ESTIMATED_LINE_LENGTH * data.size());

    Formatter<FormatTraits::CSV>::serializeHeader(buffer);
    for (const auto& tp : data) {
        const double t = (tp.stamp - lower_bound).seconds();
        Formatter<FormatTraits::CSV>::serializePoint(buffer, t, tp.point);
    }

    size_t size = buffer.size();
    if (ftruncate(fd, size) == -1) {
        close(fd);
        throw std::runtime_error("Failed to truncate file");
    }

    char* map = static_cast<char*>(mmap(nullptr, size, PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == MAP_FAILED) {
        close(fd);
        throw std::runtime_error("mmap failed");
    }

    memcpy(map, buffer.data(), size);
    munmap(map, size);
    close(fd);
}

