cmake_minimum_required(VERSION 3.8)
project(wp5_ros2_interface VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(robotiq_ft_sensor_interfaces REQUIRED)


# Add library
add_library(${PROJECT_NAME} SHARED
  src/RosInterfaceHumble.cpp
)

# Include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  yaml-cpp
  ament_index_cpp
  tf2
  tf2_ros
  tf2_eigen
  std_srvs
)

# Link yaml-cpp explicitly
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES})

# Find the ros directory dynamically, assuming it exists in the project folder
set(PROJECT_ROOT "${CMAKE_SOURCE_DIR}")
find_path(ROS_INTERFACE_PATH
    NAMES config
    PATHS "${PROJECT_ROOT}"
    NO_DEFAULT_PATH
)

if(NOT ROS_INTERFACE_PATH)
    message(FATAL_ERROR "Could not find the 'wp5_ros2_interface' directory.")
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC WP5_ROS_INTERFACE_DIR="${ROS_INTERFACE_PATH}")


# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# # Export targets
# install(
#   EXPORT ${PROJECT_NAME}-export
#   FILE ${PROJECT_NAME}Targets.cmake
#   DESTINATION lib/cmake/${PROJECT_NAME}
# )

# Add test executable
add_executable(test_ros2_interface src/test_ros2_interface.cpp)

# Pass CMake executable path to the test executable
target_compile_definitions(test_ros2_interface PUBLIC CMAKE_EXECUTABLE_PATH="${CMAKE_EXECUTABLE_PATH}")

# Link the test executable to the library
target_link_libraries(test_ros2_interface ${PROJECT_NAME})

# Link dependencies for the test executable
ament_target_dependencies(test_ros2_interface
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  yaml-cpp
  ament_index_cpp
  PCL
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  std_srvs
)

# Install the test executable
install(
  TARGETS test_ros2_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Ament export statements
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs nav_msgs visualization_msgs sensor_msgs 
  geometry_msgs Eigen3 yaml-cpp ament_index_cpp PCL pcl_conversions 
  tf2 tf2_ros tf2_eigen std_srvs )

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

# Print the CMake executable path for debugging
message(STATUS "CMake executable path: ${CMAKE_EXECUTABLE_PATH}")

# Required for ROS2 package
ament_package()


