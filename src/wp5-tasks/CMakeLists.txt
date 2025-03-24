cmake_minimum_required(VERSION 3.5)
project(wp5_tasks VERSION 0.1.0 DESCRIPTION "Task handling for robotic arms")

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify Python version for Pybind11
set(PYBIND11_PYTHON_VERSION "3.8")

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)." FORCE)
endif()

# Compile options
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Define compile-time definitions
add_compile_definitions(
    WP5_TASKS_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    PYBIND11_PYTHON_VERSION="${PYBIND11_PYTHON_VERSION}"
)

# Ensure CMake policies are compatible
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

# Find ROS 2 packages and other dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(wp5_robotic_arms REQUIRED)
find_package(wp5_ros2_interface REQUIRED)
# find_package(wp5_tools REQUIRED)
# find_package(wp5_planner REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Add utility_functions as a static library
add_library(utility_functions STATIC
    src/utilityFunctions.cpp
)

# Include directories for utility_functions
target_include_directories(utility_functions PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Add main executable
add_executable(wp5_main_task_node
    src/main_task.cpp
    src/ITaskBase.cpp
    # src/TaskSurfaceFinishing.cpp
    # src/TaskShotcrete.cpp
    src/TaskShotcreteKUL.cpp
)

# Include directories for the executable
target_include_directories(wp5_main_task_node PRIVATE
    include
    ${YAML_CPP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

# Link libraries using ament_target_dependencies to avoid conflicts
ament_target_dependencies(wp5_main_task_node
    rclcpp
    wp5_ros2_interface
    wp5_robotic_arms
    # wp5_tools
    # wp5_planner
    control_libraries
    pybind11
    yaml-cpp
    OsqpEigen
    Python3
)

# Link the utility_functions library explicitly
target_link_libraries(wp5_main_task_node utility_functions)

# Install the executable
install(TARGETS wp5_main_task_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install launch files
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Export utility_functions if other packages require it
ament_export_libraries(utility_functions)
ament_export_include_directories(include)

# Register the package with ROS 2
ament_package()
