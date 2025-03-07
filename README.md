# Trajectory Loader And Saver Nodes Using Metaprogramming and Concurrency

ROS2 nodes for collecting, visualizing, saving, and loading robot trajectory data. Contains two components:
1. **Trajectory Saver**: Collects and exports real-time trajectory
2. **Trajectory Loader**: Loads and visualizes saved trajectories

## Key Features
- **Multi-Format Support**: CSV, JSON, YAML
- **Efficient Visualization**: Double-buffered markers
- **Frame Transformation**: TF2-integrated coordinate conversion
- **Parallel Processing**: Async file I/O and transformations
- **Time-Based Filtering**: Temporal data pruning and queries

## Installation

### Dependencies
```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>visualization_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>rclcpp_components</depend>
```

### Third-Party Libraries
```bash
# Ubuntu/Debian
sudo apt-get install libyaml-cpp-dev nlohmann-json3-dev
```

### Build
```bash
mkdir -p ~/trajectory_ws/src
cd ~/trajectory_ws/src
Clone the repo
cd ..
colcon build --symlink-install
```

## Usage

### Trajectory Saver Node
```bash
ros2 launch trajectory_pkg trajectory.launch.py
```

**Save Service Call**:
```bash
ros2 service call /save_trajectory trajectory_pkg/srv/SaveTrajectory \
  "{duration: 30.0, format: 'json', file_path: '/tmp/trajectory.json'}"
```

### Trajectory Loader Node
```bash
ros2 run trajectory_pkg trajectory_loader_node \
  --ros-args \
  -p file_path:=/tmp/trajectory.json \
  -p format:=json \
  -p source_frame:=odom
```

## Parameters

### Saver Node
| Parameter       | Default | Description                          |
|-----------------|---------|--------------------------------------|
| `max_duration`  | 3600s   | Maximum history retention (seconds) |

### Loader Node
| Parameter       | Default | Description                          |
|-----------------|---------|--------------------------------------|
| `file_path`     | ""      | Path to trajectory file             |
| `format`        | csv     | File format (csv/json/yaml)         |
| `source_frame`  | map     | Original coordinate frame           |

## Services

### Saver Node
| Service           | Type                                  | Description               |
|-------------------|---------------------------------------|---------------------------|
| `/save_trajectory`| `trajectory_pkg/srv/SaveTrajectory`  | Export trajectory data    |

## Topics

### Saver Node
| Topic                 | Type                             | Description               |
|-----------------------|----------------------------------|---------------------------|
| `/odom`               | `nav_msgs/msg/Odometry`         | Input odometry            |
| `/trajectory_marker`  | `visualization_msgs/MarkerArray`| Visualization markers     |

### Loader Node
| Topic                 | Type                             | Description               |
|-----------------------|----------------------------------|---------------------------|
| `/loaded_trajectory`  | `visualization_msgs/MarkerArray`| Processed trajectory      |


## Algorithm Overview For Trajectory Saver

### 1. **Data Collection**
- **Input**: Odometry messages (`/odom`)
- **Steps**:
  1. Calculate squared speed:  
     `speed² = linear_x² + linear_y² + linear_z²`
  2. **If** `speed² < 0.0001` (stationary), ignore the data point
  3. **Else**:
     - Store current position and timestamp in `trajectory_data_`
     - Prune data older than `max_duration` (configurable parameter):
       ```cpp
       cutoff_time = current_time - max_duration
       Remove all points where timestamp < cutoff_time
       ```

### 2. **Data Storage**
- **Structure**: 
  ```cpp
  struct TrajectoryPoint {
    rclcpp::Time stamp;
    geometry_msgs::msg::Point point;
  };
  ```
- **Storage**: 
  - Time-bounded buffer (`std::vector<TrajectoryPoint>`)
  - Thread-safe access using reader-writer locks:
    - *Exclusive lock* during data addition/pruning
    - *Shared lock* during service read operations

### 3. **Visualization (Marker Publishing)**
- **Double-Buffering Technique**:
  1. `marker_front_`: Actively updated with new points in `odomCallback`
  2. `marker_back_`: Published periodically (10Hz timer)
  3. On each timer tick:
     - Swap front/back buffers
     - Publish `marker_back_` with latest points
     - Reset `marker_front_` for new data

### 4. **Data Export Service**
- **Service**: `save_trajectory` (Request: duration, format, file_path)
- **Steps**:
  1. Calculate time window:  
     `start_time = current_time - requested_duration`
  2. Extract points where `timestamp >= start_time`
  3. Export to file using format-specific writer:
     ```cpp
     if (format == "csv")  writeData<CSV>(...)
     if (format == "json") writeData<JSON>(...)
     if (format == "yaml") writeData<YAML>(...)
     ```
  4. Return success/failure with status message


### 5. **Node Initialization**  
- Set maximum trajectory retention duration from parameters  
- Create publishers/subscribers for visualization and odometry  
- Initialize two visualization buffers (front/back) with LineStrip properties  

### 6. **Odometry Input Handling**  
- Calculate squared velocity magnitude from linear components  
- Ignore stationary positions (below speed threshold)  

### 7. **Trajectory Storage**  
- Store valid positions with timestamps in time-sorted collection  
- Maintain thread safety using mutex locks during updates  

### 8. **Automatic Data Pruning**  
- Calculate cutoff time: current_time - max_duration  
- Remove all entries older than cutoff using binary search  
- Optimized for O(log n) time complexity pruning  

### 9. **Visualization Pipeline Setup**  
- Configure front buffer to receive new points in real-time  
- Prepare back buffer with existing visualization properties  

### 10. **Double-Buffer Visualization**  
- Every 100ms: Swap front/back buffers using mutex protection  
- Publish back buffer contents as persistent trajectory line  
- Reset front buffer for new incoming data collection  

### 11. **Service Request Handling**  
- Validate requested time window against current time  
- Calculate temporal filter boundary: now - requested duration  

### 12. **Data Filtering**  
- Find first position meeting time criteria via binary search  
- Extract contiguous subset of recent trajectory points  

### 13. **Format-Agnostic Serialization**  
- Initialize output structure with format-specific header  
- Process each point through conversion pipeline:  
   - Calculate relative timestamp from filter boundary  
   - Convert coordinates to text representation  

### 14. **CSV Optimization Strategy**  
- Pre-allocate memory based on estimated entry size  
- Bulk-convert points to comma-separated lines  
- Use memory-mapped files for zero-copy disk writes  

### 15. **JSON Structure Generation**  
- Build array of JSON objects with time/position properties  
- Maintain proper comma separation between entries  
- Enclose complete dataset in top-level array brackets  

### 16. **YAML Hierarchy Construction**  
- Create nested list structure with indentation  
- Represent each point as separate YAML object  
- Maintain key-value formatting for coordinates  

### 17. **Error Handling**  
- Detect empty dataset before file operations  
- Validate supported output formats  
- Catch filesystem errors during write operations  

### 18. **Resource Management**  
- Ensure proper file descriptor closure  
- Release memory-mapped resources after writes  
- Maintain lock durations for concurrency safety  

### 19. **Concurrency Model**  
- Use shared locks for read-only service operations  
- Apply exclusive locks during data modification  
- Synchronize buffer swaps with visualization updates  

This algorithm focuses on temporal efficiency (O(log n) pruning), memory optimization (buffer swapping), and format-specific serialization while maintaining thread safety through strategic locking mechanisms.


# Algorithm Overview For Trajectory Loading Node

### 1. **Node Initialization**  
- Read parameters: file path, data format (CSV/JSON/YAML), and source coordinate frame  
- Initialize TF2 components for coordinate transformations  
- Create visualization marker publisher and periodic update timer

### 2. **Memory-Mapped File Handling**  
- Open specified file using low-level POSIX file descriptors  
- Map entire file contents to memory for zero-copy access  
- Automatically release resources using RAII pattern

### 3. **Asynchronous Data Loading**  
- Launch parallel thread for file parsing  
- Maintain main thread responsiveness during I/O operations

### 4. **Format Detection & Parser Selection**  
- Choose appropriate parser based on file extension/format parameter  
- Instantiate CSV, JSON, or YAML parsing strategy

### 5. **Header Processing**  
- Skip CSV header line ("time,x,y,z")  
- Verify JSON/YAML root structure compatibility  
- Handle format-specific metadata if present

### 6. **Coordinate Extraction**  
- CSV: Parse comma-separated values line-by-line  
- JSON: Process array of position objects  
- YAML: Read nested trajectory list entries  
- Convert text values to floating-point coordinates

### 7. **Error-Resilient Parsing**  
- Catch number format exceptions during conversion  
- Skip malformed entries with error logging  
- Maintain partial results from valid data

### 8. **Frame Transformation Setup**  
- Prepare TF2 transform buffer with timeout  
- Configure source→target frame mapping (e.g., map→odom)

### 9. **Batch Transformation Pipeline**  
- Create parallel transformation tasks for each point  
- Use async/await pattern for concurrent processing

### 10. **Coordinate System Conversion**  
- Convert raw points to PoseStamped messages  
- Apply latest available transform from TF tree  
- Handle temporal synchronization via transform timestamp

### 11. **Transformation Error Handling**  
- Catch and log TF lookup exceptions  
- Filter out points with failed transformations  
- Maintain visualization continuity with partial data

### 12. **Visualization Marker Construction**  
- Configure LINE_STRIP marker properties (color, thickness)  
- Set odom frame as visualization reference  
- Pre-allocate memory for efficient point storage

### 13. **Result Aggregation**  
- Collect successfully transformed points  
- Maintain original trajectory ordering  
- Discard points with transformation failures

### 14. **Periodic Visualization Update**  
- Publish marker array at fixed 2Hz interval  
- Ensure fresh TF data usage through timestamp updates  
- Reuse parsed data between publications

### 15. **Resource Management**  
- Automatically unmap memory-mapped files  
- Cleanup async task resources  
- Gracefully handle node shutdown

Key Features:  
- **Efficient I/O**: Memory mapping avoids double-buffering  
- **Parallel Processing**: Concurrent coordinate transformations  
- **Format Agnosticism**: Unified interface for multiple file types  
- **Temporal Decoupling**: Periodic updates independent of loading  
- **Frame Flexibility**: Dynamic coordinate system adaptation
