#include "RosInterfaceHumble.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

using namespace std;

RosInterfaceHumble::RosInterfaceHumble(const std::string &robotName) :
    Node("ros_interface_humble"), robotName_(robotName), nJoint_(0), shotcreteBegin_(false), shotcreteDone_(false) {
  try {
    dataPath_ = "/home/robetarme_user/ros2_ws";
    reference_frame_ = "base_link"; // Default reference frame

    // Locate YAML Configuration File
    string alternativeYamlPath = string(WP5_ROS_INTERFACE_DIR) + "/config/ros_interface_config.yaml";
    string yamlPath = dataPath_ + "/config/ros_interface_config.yaml";
    string nameYaml = "colcon_ws/config/ros_interface_config.yaml";
    // Check YAML file existence
    ifstream originalFile(yamlPath);
    if (originalFile.good()) {
      RCLCPP_INFO(this->get_logger(), "Using general YAML file: %s", nameYaml.c_str());
    } else {
      yamlPath = alternativeYamlPath;
      RCLCPP_WARN(this->get_logger(), "Using local YAML file: %s", yamlPath.c_str());
    }

    YAML::Node config = YAML::LoadFile(yamlPath);
    YAML::Node robotNode = config[robotName_];

    // Load Robot-Specific Parameters
    nJoint_ = robotNode["number_joint"].as<int>();
    // std::string FTTopic = robotNode["ft_topic"].as<std::string>();
    std::string actualStateTopic = robotNode["joint_topic"].as<std::string>();
    std::string commandStateTopic = robotNode["joint_command"].as<std::string>();
    std::string actuatorSprayTopic = robotNode["actuator_spray_topic"].as<std::string>();
    std::string topicTwistKulCmd = robotNode["topic_twist_kul_cmd"].as<std::string>();
    std::string topicTwistKulState = robotNode["topic_twist_kul"].as<std::string>();
    std::string topicPoseKulState = robotNode["topic_pose_kul"].as<std::string>();
    std::string topicCamera = robotNode["topic_camera"].as<std::string>();
    std::string topicStl = robotNode["topic_mesh"].as<std::string>();
    // std::string robot_description_param = robotNode["robot_description_param"].as<std::string>();

    // Initialize Data Containers
    jointsPosition_.assign(nJoint_, 0.0);
    jointsVelocity_.assign(nJoint_, 0.0);
    jointsEffort_.assign(nJoint_, 0.0);
    firstQuatPos_ = std::vector<double>(7, 0.0);

    // Initialize Service Server
    initShotcreteService_ = this->create_service<std_srvs::srv::Trigger>(
        "/integration/initShotcrete",
        std::bind(
            &RosInterfaceHumble::initShotcreteServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    startShotcreteService_ = this->create_service<std_srvs::srv::Trigger>(
        "/integration/startShotcrete",
        std::bind(
            &RosInterfaceHumble::startShotcreteServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    stopShotcreteService_ = this->create_service<std_srvs::srv::Trigger>(
        "/integration/stopShotcrete",
        std::bind(
            &RosInterfaceHumble::stopShotcreteServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Subscribers
    // subFtSensor_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    //     FTTopic, 10, std::bind(&RosInterfaceHumble::ftCallback, this, std::placeholders::_1));
    subStateJoints_ = this->create_subscription<sensor_msgs::msg::JointState>(
        actualStateTopic, 10, std::bind(&RosInterfaceHumble::jointStateCallback, this, std::placeholders::_1));
    subTwistKUL = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topicTwistKulCmd, 10, std::bind(&RosInterfaceHumble::twistKULCallback, this, std::placeholders::_1));
    pointcloudTransformedCropSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topicCamera, 10, std::bind(&RosInterfaceHumble::pointCloudCallback, this, std::placeholders::_1));
    subFirstPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/integration/firstPose", 10, std::bind(&RosInterfaceHumble::firstPoseCallback, this, std::placeholders::_1));
    // sharedControlSub_ = this->create_subscription<std_msgs::msg::Bool>(
    //     "/shared_control", 10, std::bind(&RosInterfaceHumble::sharedControlCallback, this, std::placeholders::_1));
    // pos_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //     "/waypoint_list", 10, std::bind(&PolygonCoverage::poseArrayCallback, this, std::placeholders::_1));

    // Publishers
    pubActualCartesianPoseEEF_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topicPoseKulState, 10);
    pubActualCartesianTwistEEF_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topicTwistKulState, 10);
    pubStateJoints_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(commandStateTopic, 10);
    polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/original_polygon", 10);
    path_pub_flat_ = this->create_publisher<nav_msgs::msg::Path>("/result_path_flat", 10);
    path_pub_final_ = this->create_publisher<nav_msgs::msg::Path>("/result_path_final", 10);
    stl_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topicStl, 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/first_waypoint", 10);

    pubShotcreteBegin_ = this->create_publisher<std_msgs::msg::Bool>("/integration/firstPoseAchieved", 10);
    pubShotcreteDone_ = this->create_publisher<std_msgs::msg::Bool>("/integration/shotcreteDone", 10);
    // pubSharedControl_ = this->create_publisher<std_msgs::msg::Bool>("/shared_control", 10);
    // Initialize tf2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Initialized ROS2 Interface for robot: %s", robotName_.c_str());

    save_requested_ = false;
    prevFilteredJointSpeed_ = std::vector<double>(nJoint_, 0.0);
    filterON_ = true;
    firstPosReceived_ = false;
    std::string urdf_path = "/home/robetarme_user/ros2_ws/src/wp5-robotic-arms/urdf/" + robotName_ + ".urdf";
    // saveURDFToFile(urdf_path, robot_description_param);

  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception caught: %s", e.what());
  }
}

void RosInterfaceHumble::startShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  // Check if the message contains "true" or "false"
  if (firstTwistReceived_ && firstPosReached_) {
    shotcreteBegin_ = true;
  } else if (!firstTwistReceived_) {
    shotcreteBegin_ = false;
    std::cout << "No twist received yet." << std::endl;
  } else if (!firstPosReached_) {
    shotcreteBegin_ = false;
    std::cout << "Working position not reached" << std::endl;
  } else {
    shotcreteBegin_ = false;
  }

  // Process trigger_value here (e.g., control the shotcrete mechanism)
  response->success = shotcreteBegin_;
  response->message = shotcreteBegin_ ? "Shotcrete ON" : "Shotcrete OFF";
}

void RosInterfaceHumble::stopShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  shotcreteDone_ = true;

  // Process trigger_value here (e.g., control the shotcrete mechanism)
  response->success = shotcreteDone_;
  response->message = shotcreteDone_ ? "Shotcrete OFF" : "Shotcrete ON";
}
void RosInterfaceHumble::saveURDFToFile(std::string file_path, std::string param_name) {
  // Fetch the URDF parameter
  std::cout << "Fetching parameter: " << param_name << std::endl;

  if (!this->has_parameter(param_name)) {
    RCLCPP_ERROR(this->get_logger(), "Parameter '%s' not found!", param_name.c_str());
    return;
  }

  std::string robot_description = this->get_parameter(param_name).as_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR(this->get_logger(), "URDF parameter '%s' is empty.", param_name.c_str());
    return;
  }

  // Check if the robot_description is empty
  if (robot_description.empty()) {
    RCLCPP_WARN(this->get_logger(), "URDF not found in parameters.");
    return;
  }

  // Optionally, print a portion of the URDF to verify
  std::cout << "URDF content snippet: " << robot_description.substr(0, 500) << "..." << std::endl;

  // If the URDF exists, save it to a file
  std::ofstream urdf_file(file_path);

  if (urdf_file.is_open()) {
    urdf_file << robot_description;
    urdf_file.close();
    RCLCPP_INFO(this->get_logger(), "URDF saved to '%s'", file_path.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing URDF.");
  }
}

void RosInterfaceHumble::firstPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety

  // Set the flag indicating the first pose has been received
  firstQuatPos_ = {msg->pose.orientation.w,
                   msg->pose.orientation.x,
                   msg->pose.orientation.y,
                   msg->pose.orientation.z,
                   msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z};
}

void RosInterfaceHumble::initShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void) request; // Avoid unused parameter warning
  RCLCPP_INFO(this->get_logger(), "Inittialization Shotcrete Service Called");

  if (firstQuatPos_ == std::vector<double>(7, 0.0)) {
    RCLCPP_ERROR(this->get_logger(), "First pose not received yet.");
    response->success = false;
    response->message = "First pose not received yet.";
    firstPosReceived_ = false;
  } else {
    firstPosReceived_ = true;
    response->success = true;
    response->message = "Got the first pose.";
  }
}

// bool RosInterfaceHumble::callRobotiqService(uint8_t command_id = 8) {
//   auto request = std::make_shared<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request>();
//   request->command_id = command_id;
//   bool checkResult = false;

//   auto future = clientFtsensor_->async_send_request(request);
//   auto result = rclcpp::spin_until_future_complete(this->shared_from_this(), future);

//   if (result == rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_INFO(this->get_logger(), "Service Response: %s", future.get()->res.c_str());
//     checkResult = true;
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
//     checkResult = false;
//   }
//   return checkResult;
// }

bool RosInterfaceHumble::readSTL(const std::string &file_path, std::vector<geometry_msgs::msg::Point> &vertices) {
  std::ifstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("stl_publisher"), "Failed to open STL file: %s", file_path.c_str());
    return false;
  }

  // Skip header (80 bytes)
  char header[80];
  file.read(header, 80);

  // Read number of triangles (4 bytes)
  uint32_t num_triangles;
  file.read(reinterpret_cast<char *>(&num_triangles), sizeof(num_triangles));

  for (uint32_t i = 0; i < num_triangles; ++i) {
    // Skip normal vector (12 bytes)
    file.seekg(12, std::ios::cur);

    // Read vertices (3 vertices * 3 floats = 36 bytes)
    for (int v = 0; v < 3; ++v) {
      float x, y, z;
      file.read(reinterpret_cast<char *>(&x), sizeof(float));
      file.read(reinterpret_cast<char *>(&y), sizeof(float));
      file.read(reinterpret_cast<char *>(&z), sizeof(float));

      geometry_msgs::msg::Point vertex;
      vertex.x = x;
      vertex.y = y;
      vertex.z = z;
      vertices.push_back(vertex);
    }

    // Skip attribute byte count (2 bytes)
    file.seekg(2, std::ios::cur);
  }

  file.close();
  return true;
}

void RosInterfaceHumble::publishSTL() {

  // Load STL File
  std::string stlPath = dataPath_ + "/data/meshes/mesh_target_transformed_simplified.stl";
  if (!readSTL(stlPath, vertices_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load STL file.");
  }
  if (vertices_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No vertices to publish.");
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = reference_frame_; // Set your desired frame_id
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "stl_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // Add vertices to the marker
  for (size_t i = 0; i < vertices_.size(); i += 3) {
    marker.points.push_back(vertices_[i]);
    marker.points.push_back(vertices_[i + 1]);
    marker.points.push_back(vertices_[i + 2]);
  }

  stl_pub_->publish(marker);
}
void RosInterfaceHumble::publishPoseFirst() {
  std::string txtPath = dataPath_ + "/data/paths/waypoints_in_original_space.txt";
  std::ifstream file(txtPath);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open TXT file: %s", txtPath.c_str());
    return;
  }

  std::string line;
  std::getline(file, line); // Skip the header
  if (!std::getline(file, line)) {
    RCLCPP_ERROR(this->get_logger(), "TXT file is empty or missing data.");
    return;
  }
  file.close();

  std::stringstream ss(line);
  std::vector<std::string> tokens;
  std::string token;

  while (std::getline(ss, token, ',')) {
    tokens.push_back(token);
  }

  if (tokens.size() != 7) {
    RCLCPP_ERROR(this->get_logger(), "Invalid format in TXT file.");
    return;
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = this->get_clock()->now();
  pose_stamped.header.frame_id = "world"; // Change this to your relevant frame

  pose_stamped.pose.position.x = std::stod(tokens[0]);
  pose_stamped.pose.position.y = std::stod(tokens[1]);
  pose_stamped.pose.position.z = std::stod(tokens[2]);
  pose_stamped.pose.orientation.w = std::stod(tokens[3]);
  pose_stamped.pose.orientation.x = std::stod(tokens[4]);
  pose_stamped.pose.orientation.y = std::stod(tokens[5]);
  pose_stamped.pose.orientation.z = std::stod(tokens[6]);

  pose_pub_->publish(pose_stamped);
}

// Function to spin_some
void RosInterfaceHumble::spinNode() { rclcpp::spin_some(shared_from_this()); }

nav_msgs::msg::Path RosInterfaceHumble::convertFileToNavMsgsPath() {
  std::string file_path = dataPath_ + "/data/paths/waypoints_in_original_space.txt";
  std::string frame_id = reference_frame_;
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;

  std::ifstream infile(file_path);
  if (!infile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
    return path;
  }

  std::string line;

  // Skip the first line if it starts with '#'
  if (std::getline(infile, line) && line[0] == '#') {
    // RCLCPP_INFO(this->get_logger(), "Skipping header line: %s", line.c_str());
  }

  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::vector<std::string> tokens;
    std::string token;

    // Split line by comma
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    // Ensure we have exactly 7 values (px, py, pz, qw, qx, qy, qz)
    if (tokens.size() != 7) {
      RCLCPP_WARN(this->get_logger(), "Invalid line format: %s", line.c_str());
      continue;
    }

    try {
      double px = std::stod(tokens[0]);
      double py = std::stod(tokens[1]);
      double pz = std::stod(tokens[2]);
      double qw = std::stod(tokens[3]);
      double qx = std::stod(tokens[4]);
      double qy = std::stod(tokens[5]);
      double qz = std::stod(tokens[6]);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = frame_id;
      pose_stamped.header.stamp = this->now();

      pose_stamped.pose.position.x = px;
      pose_stamped.pose.position.y = py;
      pose_stamped.pose.position.z = pz;
      pose_stamped.pose.orientation.w = qw;
      pose_stamped.pose.orientation.x = qx;
      pose_stamped.pose.orientation.y = qy;
      pose_stamped.pose.orientation.z = qz;

      path.poses.push_back(pose_stamped);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Error parsing line: %s - Exception: %s", line.c_str(), e.what());
      continue;
    }
  }

  infile.close();
  return path;
}

geometry_msgs::msg::PolygonStamped RosInterfaceHumble::convertPolygonToPolygonRosMsg() {
  std::string file_path = dataPath_ + "/data/boundary/boundary_in_original_space.txt";
  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.stamp = this->now();
  polygon_msg.header.frame_id = reference_frame_;

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }

  std::string line;
  bool header_skipped = false;
  while (std::getline(file, line)) {
    // Skip header if exists
    if (line[0] == '#') {
      continue;
    }

    std::stringstream ss(line);
    geometry_msgs::msg::Point32 point;
    char delimiter;

    if (!(ss >> point.x >> delimiter >> point.y >> delimiter >> point.z)) {
      throw std::runtime_error("Invalid file format: " + line);
    }

    polygon_msg.polygon.points.push_back(point);
  }

  file.close();
  return polygon_msg;
}
void RosInterfaceHumble::publishNavmsg(const nav_msgs::msg::Path &path) { path_pub_final_->publish(path); }
void RosInterfaceHumble::publishPolygon(const geometry_msgs::msg::PolygonStamped &polygonMsg) {
  polygon_pub_->publish(polygonMsg);
}

std::vector<std::vector<double>> RosInterfaceHumble::convertNavPathToVectorVector(
    const nav_msgs::msg::Path &inputPath) {
  size_t size = inputPath.poses.size();
  std::vector<std::vector<double>> path;
  // std::string file_path = std::string(WP5_ROS_INTERFACE_DIR) + "/data/paths/waypointInOriSpaceConverted.txt";
  std::string file_path = dataPath_ + "/data/paths/waypointInOriSpaceConverted.txt";

  std::ofstream outFile(file_path);
  if (!outFile) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file for writing: %s", file_path.c_str());
    return path;
  }

  for (size_t i = 0; i < size; i++) {
    geometry_msgs::msg::PoseStamped pose = inputPath.poses[i];
    std::vector<double> quatPos;

    // Extract quaternion orientation
    quatPos.push_back(pose.pose.orientation.x);
    quatPos.push_back(pose.pose.orientation.y);
    quatPos.push_back(pose.pose.orientation.z);
    quatPos.push_back(pose.pose.orientation.w);

    // Extract position
    quatPos.push_back(pose.pose.position.x);
    quatPos.push_back(pose.pose.position.y);
    quatPos.push_back(pose.pose.position.z);

    // Add to path
    path.push_back(quatPos);

    // Write to file
    outFile << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << " "
            << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " "
            << pose.pose.orientation.w << std::endl;
  }

  outFile.close();
  return path;
}

// // THREAD-SAFE CALLBACKS
// void RosInterfaceHumble::ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
//   std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
//   if (msg->wrench.force.x != 0.0 || msg->wrench.force.y != 0.0 || msg->wrench.force.z != 0.0) {
//     wrenchSensor_ = {msg->wrench.force.x,
//                      msg->wrench.force.y,
//                      msg->wrench.force.z,
//                      msg->wrench.torque.x,
//                      msg->wrench.torque.y,
//                      msg->wrench.torque.z};
//     // RCLCPP_INFO(this->get_logger(), "FT Data Received.");
//     for (size_t i = 0; i < wrenchSensor_.size(); ++i) {
//       wrenchSensor_[i] -= wrenchBias_[i];
//     }
//   }
// }
void RosInterfaceHumble::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety

  int num_clouds_to_accumulate_ = 10; // Adjust as needed
  int max_clouds_in_memory_ = 10;     // Maximum number of clouds allowed in memory

  // Convert PointCloud2 message to PCL point cloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Accumulate the point clouds
  accumulated_clouds_.push_back(cloud);

  // Ensure that the number of clouds in memory does not exceed the maximum limit
  if (accumulated_clouds_.size() > max_clouds_in_memory_) {
    accumulated_clouds_.erase(accumulated_clouds_.begin()); // Remove the oldest point cloud
  }

  // Check if we've accumulated enough clouds for averaging and processing
  if (accumulated_clouds_.size() >= num_clouds_to_accumulate_ && save_requested_) {

    // Create a point cloud to store the averaged result
    pcl::PointCloud<pcl::PointXYZ>::Ptr averaged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    averaged_cloud->resize(accumulated_clouds_[0]->size()); // Assuming all clouds are the same size

    // Loop through each point and compute the average for each dimension
    for (size_t i = 0; i < averaged_cloud->size(); ++i) {
      pcl::PointXYZ avg_point;
      avg_point.x = 0;
      avg_point.y = 0;
      avg_point.z = 0;

      // Sum all the points at the same index from each accumulated cloud
      for (const auto &pc : accumulated_clouds_) {
        avg_point.x += pc->points[i].x;
        avg_point.y += pc->points[i].y;
        avg_point.z += pc->points[i].z;
      }

      // Compute the average by dividing the sum by the number of clouds
      avg_point.x /= accumulated_clouds_.size();
      avg_point.y /= accumulated_clouds_.size();
      avg_point.z /= accumulated_clouds_.size();

      averaged_cloud->points[i] = avg_point;
    }

    // Save the averaged and filtered point cloud to a PLY file
    std::string file_path = dataPath_ + "/data/pointclouds/pointcloud_target_transformed.ply";
    std::filesystem::path directory = std::filesystem::path(file_path).parent_path();

    // Check and create the directory if it doesn't exist
    if (!std::filesystem::exists(directory)) {
      if (std::filesystem::create_directories(directory)) {
        RCLCPP_INFO(rclcpp::get_logger("PolygonCoverage"), "Created directory: %s", directory.c_str());
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("PolygonCoverage"), "Failed to create directory: %s", directory.c_str());
        return;
      }
    }

    if (pcl::io::savePLYFile(file_path, *averaged_cloud) == 0) {
      RCLCPP_INFO(rclcpp::get_logger("PolygonCoverage"),
                  "Averaged and filtered point cloud saved as pointcloud_target_transformed.ply");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PolygonCoverage"),
                   "Failed to save averaged and filtered point cloud to %s",
                   file_path.c_str());
    }

    // Reset the accumulation (or you could continue accumulating and averaging)
    accumulated_clouds_.clear();
    save_requested_ = false;
  }
}

void RosInterfaceHumble::getPointCloud() {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
  //clean the accumulated clouds
  accumulated_clouds_.clear();
  auto loopRate = std::make_unique<rclcpp::Rate>(10); // Create the rate object once
  save_requested_ = true;

  while (save_requested_ && rclcpp::ok()) {
    spinNode();
    loopRate->sleep();
  }
}

void RosInterfaceHumble::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety

  if (msg->position.empty())
    return;

  jointsPosition_ = msg->position;
  jointsVelocity_ = msg->velocity;
  jointsEffort_ = msg->effort;

  // Check if the robot is "ur5_robot"
  if (robotName_ == "ur5_robot") {
    // Define the desired order of joints for UR5
    const std::vector<std::string> ur5_joint_order = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    // Map joint names to their indices for fast lookup
    std::unordered_map<std::string, size_t> joint_indices;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_indices[msg->name[i]] = i;
    }

    // Reserve memory for efficiency
    std::vector<double> reordered_position, reordered_velocity, reordered_effort;
    reordered_position.reserve(ur5_joint_order.size());
    reordered_velocity.reserve(ur5_joint_order.size());
    reordered_effort.reserve(ur5_joint_order.size());

    // Reorder joint states based on the desired order
    for (const auto &joint : ur5_joint_order) {
      auto it = joint_indices.find(joint);
      if (it != joint_indices.end()) {
        size_t index = it->second;
        reordered_position.push_back(index < msg->position.size() ? msg->position[index] : 0.0);
        reordered_velocity.push_back(index < msg->velocity.size() ? msg->velocity[index] : 0.0);
        reordered_effort.push_back(index < msg->effort.size() ? msg->effort[index] : 0.0);
      } else {
        // If a joint is missing, push default values
        reordered_position.push_back(0.0);
        reordered_velocity.push_back(0.0);
        reordered_effort.push_back(0.0);
      }
    }

    // Update the member variables with reordered data
    jointsPosition_ = std::move(reordered_position);
    jointsVelocity_ = std::move(reordered_velocity);
    jointsEffort_ = std::move(reordered_effort);
  }

  // Check if the robot is "robetarme"
  if (robotName_ == "robetarme") {
    // Define the joint names we want to keep (Joint_0 to Joint_6)
    const std::vector<std::string> required_joints = {
        "Joint_0", "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"};

    // Initialize new containers to store the filtered joint states
    std::vector<double> filtered_position, filtered_velocity, filtered_effort;

    // Reserve space for efficiency
    filtered_position.reserve(required_joints.size());
    filtered_velocity.reserve(required_joints.size());
    filtered_effort.reserve(required_joints.size());

    // Go through the message and select only the required joints
    for (const auto &joint_name : required_joints) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
      if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it); // Find the index of the joint
        filtered_position.push_back(msg->position[index]);
        filtered_velocity.push_back(msg->velocity[index]);
        filtered_effort.push_back(msg->effort[index]);
      } else {
        // If a joint is missing, push default values (optional)
        filtered_position.push_back(0.0);
        filtered_velocity.push_back(0.0);
        filtered_effort.push_back(0.0);
      }
    }

    // Update the member variables with filtered data
    jointsPosition_ = std::move(filtered_position);
    jointsVelocity_ = std::move(filtered_velocity);
    jointsEffort_ = std::move(filtered_effort);
  }
}

void RosInterfaceHumble::twistKULCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
  firstTwistReceived_ = true;

  twistKUL_ = Eigen::VectorXd::Zero(6); // Resize the vector to hold 6 elements
  twistKUL_(0) = msg->twist.linear.x;
  twistKUL_(1) = msg->twist.linear.y;
  twistKUL_(2) = msg->twist.linear.z;
  twistKUL_(3) = msg->twist.angular.x;
  twistKUL_(4) = msg->twist.angular.y;
  twistKUL_(5) = msg->twist.angular.z;
}

// void RosInterfaceHumble::sharedControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
//   std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
//   sharedControl_ = msg->data;
//   RCLCPP_INFO(this->get_logger(), "Shared Control: %s", sharedControl_ ? "ON" : "OFF");
// }

// SAFE GETTERS
Eigen::VectorXd RosInterfaceHumble::getTwistKUL() {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
  return twistKUL_;
}

tuple<vector<double>, vector<double>, vector<double>> RosInterfaceHumble::receiveState() {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
  return make_tuple(jointsPosition_, jointsVelocity_, jointsEffort_);
}
// std::vector<double> RosInterfaceHumble::receiveWrench() {
//   std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
//   return wrenchSensor_;
// }
// void RosInterfaceHumble::setWrenchBias(const std::vector<double> &wrenchBias) {
//   std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
//   wrenchBias_ = wrenchBias;
// }

// SENDERS
void RosInterfaceHumble::sendState(const std::vector<double> &data) {
  std::vector<double> smoothedData = data;

  // Apply exponential moving average to each joint speed with dynamic alpha based on the error
  if (!prevFilteredJointSpeed_.empty() && filterON_) {
    for (size_t i = 0; i < data.size(); ++i) {
      // Calculate the error (absolute difference between the current data and the previous filtered value)
      double error = std::abs(data[i] - prevFilteredJointSpeed_[i]);
      int k = 4;
      // Calculate alpha using arctangent of the error
      double alpha = std::exp(-k * error);
      if (alpha < 0.1) {
        alpha = 0.1;
      }
      if (alpha > 1.0) {
        alpha = 1.0;
      }

      // Apply exponential moving average with dynamic alpha
      smoothedData[i] = alpha * data[i] + (1 - alpha) * prevFilteredJointSpeed_[i];
    }
  }

  // Store the smoothed data for the next iteration
  prevFilteredJointSpeed_ = smoothedData;

  // Publish the smoothed state
  std_msgs::msg::Float64MultiArray nextJointMsg;
  nextJointMsg.data = smoothedData;

  pubStateJoints_->publish(nextJointMsg);
}

void RosInterfaceHumble::setActualCartesianTwistEEF(Eigen::VectorXd &data) {
  if (data.size() != 6) {
    RCLCPP_ERROR(this->get_logger(), "Input Eigen::VectorXd size must be 6 to set Twist.");
    return;
  }

  geometry_msgs::msg::TwistStamped twistStampedMsg;
  twistStampedMsg.header.stamp = this->now();
  twistStampedMsg.header.frame_id = reference_frame_; // Default frame ID

  twistStampedMsg.twist.linear.x = data[0];
  twistStampedMsg.twist.linear.y = data[1];
  twistStampedMsg.twist.linear.z = data[2];
  twistStampedMsg.twist.angular.x = data[3];
  twistStampedMsg.twist.angular.y = data[4];
  twistStampedMsg.twist.angular.z = data[5];

  pubActualCartesianTwistEEF_->publish(twistStampedMsg);
}
void RosInterfaceHumble::setActualCartesianPoseEEF(const std::pair<Eigen::Quaterniond, Eigen::Vector3d> &data) {
  // Ensure the quaternion is valid
  const Eigen::Quaterniond &quat = data.first;
  if (std::abs(quat.norm() - 1.0) > 1e-6) {
    RCLCPP_ERROR(this->get_logger(), "Quaternion is not normalized. Ensure it is valid.");
    return;
  }

  geometry_msgs::msg::PoseStamped poseStampedMsg;

  // Add timestamp and frame_id (use configurable frame if applicable)
  poseStampedMsg.header.stamp = this->now();
  poseStampedMsg.header.frame_id = reference_frame_; // Default frame ID, can be made configurable

  // Assign orientation (quaternion)
  poseStampedMsg.pose.orientation.x = quat.x();
  poseStampedMsg.pose.orientation.y = quat.y();
  poseStampedMsg.pose.orientation.z = quat.z();
  poseStampedMsg.pose.orientation.w = quat.w();

  // Assign position
  const Eigen::Vector3d &pos = data.second;
  poseStampedMsg.pose.position.x = pos.x();
  poseStampedMsg.pose.position.y = pos.y();
  poseStampedMsg.pose.position.z = pos.z();

  // Publish the PoseStamped message
  pubActualCartesianPoseEEF_->publish(poseStampedMsg);
}

std::vector<double> RosInterfaceHumble::getWorkingPose() {
  std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
  return firstQuatPos_;
}

Eigen::Affine3d RosInterfaceHumble::getTransform(const std::string &target_frame, const std::string &source_frame) {
  try {
    // Lookup transform from source_frame to target_frame
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    // RCLCPP_INFO(this->get_logger(), "Transform successfully retrieved.");
    return transformToEigen(transform_stamped);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    throw; // Rethrow exception so the caller knows the transform was unavailable
  }
}

Eigen::Affine3d RosInterfaceHumble::transformToEigen(const geometry_msgs::msg::TransformStamped &transform) {
  Eigen::Affine3d eigen_transform;
  // Translation
  eigen_transform.translation() << transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z;

  // Rotation (quaternion to rotation matrix)
  Eigen::Quaterniond quaternion(transform.transform.rotation.w,
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z);

  eigen_transform.linear() = quaternion.toRotationMatrix();

  return eigen_transform;
}

// bool RosInterfaceHumble::getSharedControl() {
//   std::lock_guard<std::mutex> lock(dataMutex_); // Thread safety
//   return sharedControl_;
// }

// void RosInterfaceHumble::setSharedControl(bool &data) {
//   sharedControl_ = data;
//   // Create a message of type std_msgs::msg::Bool
//   std_msgs::msg::Bool msg;
//   msg.data = data; // Set the data field
//   // Publish the message using the publisher
//   pubSharedControl_->publish(msg);
// }

bool RosInterfaceHumble::readyToWorkingPosition() { return firstPosReceived_; }
bool RosInterfaceHumble::isShotcreteReadyToBegin() { return shotcreteBegin_; }
bool RosInterfaceHumble::isShotcreteDone() { return shotcreteDone_; }
bool RosInterfaceHumble::isFirstPosReached() { return firstPosReached_; }

void RosInterfaceHumble::setShotcreteDone(bool &data) {
  shotcreteDone_ = data;

  // Create a message of type std_msgs::msg::Bool
  std_msgs::msg::Bool msg;
  msg.data = data; // Set the data field

  // Publish the message using the publisher
  pubShotcreteDone_->publish(msg);
}
void RosInterfaceHumble::setShotcreteBegin(bool &data) {
  shotcreteBegin_ = data;

  // Create a message of type std_msgs::msg::Bool
  std_msgs::msg::Bool msg;
  msg.data = data; // Set the data field

  // Publish the message using the publisher
  pubShotcreteBegin_->publish(msg);
}

void RosInterfaceHumble::setFirstPosReached(bool &data) { firstPosReached_ = data; }
