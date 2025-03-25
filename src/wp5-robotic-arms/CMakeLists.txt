cmake_minimum_required(VERSION 3.5)
project(wp5_robotic_arms VERSION 0.1.0 DESCRIPTION "Drivers library to manage different robotic arms")

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)." FORCE)
endif()

# Compile options
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Define compile-time definitions
add_compile_definitions(
    WP5_ROBOTIC_ARMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    PYBIND11_PYTHON_VERSION="3.8"
)

# Add external libraries
add_subdirectory(libs/ik-geo-cpp EXCLUDE_FROM_ALL)
add_subdirectory(libs/trac-ik/trac_ik_lib)

# Find ROS 2 packages and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_libraries REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_package(robetarme_description REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Create the main library
add_library(${PROJECT_NAME} SHARED
    src/IRoboticArmBase.cpp
    src/RoboticArmCobod.cpp
)

# Set properties for the library
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    CXX_STANDARD 17
)

# Add include directories for the library
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/libs/ik-geo-cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/trac-ik/trac_ik_lib/include
    ${YAML_CPP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

# Link libraries to the main library
target_link_libraries(${PROJECT_NAME} PRIVATE
    yaml-cpp
    ${EIGEN3_LIBRARIES}
    ${control_libraries_LIBRARIES}
    OsqpEigen::OsqpEigen
    # ik_geo
    trac_ik
    rclcpp::rclcpp
    pybind11::embed
    ${Python3_LIBRARIES}
)

# Install the main library
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the ik_geo library manually
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/libs/ik-geo-cpp/rust-wrapper/target/release/libik_geo.so
#     DESTINATION lib
#     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
# )

# Install directories
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf
        FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro" PATTERN "*.*" EXCLUDE)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY include/ DESTINATION include)

# Install external library includes
# install(DIRECTORY libs/ik-geo-cpp/include/
#     DESTINATION include/libs/ik-geo-cpp)
install(DIRECTORY libs/trac-ik/trac_ik_lib/include/
    DESTINATION include/libs/trac-ik)

# Export dependencies
ament_export_dependencies(
    rclcpp
    rclpy
    yaml-cpp
    Eigen3
    control_libraries
    pybind11
    Python3
)

# Export include directories
ament_export_include_directories(
    include
    # include/libs/ik-geo-cpp
    include/libs/trac-ik
)

# Export libraries
ament_export_libraries(${PROJECT_NAME}  trac_ik)

# Register the package with ROS 2
ament_package()
