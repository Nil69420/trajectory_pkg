cmake_minimum_required(VERSION 3.8)
project(trajectory_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fetch yaml-cpp
include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Use a specific tag, commit hash, or branch name
)
FetchContent_MakeAvailable(yaml-cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp_components REQUIRED)

# Generate ROS 2 interfaces
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

# Add trajectory_node executable
add_executable(trajectory_node
    src/trajectory_node.cpp
    src/formatter.cpp
    src/trajectory_main.cpp
)

# Add trajectory_loader_node executable
add_executable(trajectory_loader_node
    src/trajectory_loader_node.cpp
    src/trajectory_loader_main.cpp
)

# Link dependencies for trajectory_node
ament_target_dependencies(trajectory_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link dependencies for trajectory_loader_node
ament_target_dependencies(trajectory_loader_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  nlohmann_json
  rclcpp_components
)

# Explicitly link yaml-cpp to trajectory_loader_node
target_link_libraries(trajectory_loader_node yaml-cpp)

# Link ROS 2 typesupport for trajectory_node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_node "${cpp_typesupport_target}")

# Link ROS 2 typesupport for trajectory_loader_node
target_link_libraries(trajectory_loader_node "${cpp_typesupport_target}")

# Include directories
target_include_directories(trajectory_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(trajectory_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
    trajectory_node
    trajectory_loader_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install include and launch directories
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()