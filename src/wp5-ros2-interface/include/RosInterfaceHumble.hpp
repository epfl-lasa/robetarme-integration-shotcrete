/**
 * @file RosInterfaceHumble.hpp
 * @brief Create a ROS2 interface to communicate with the robotic arm
 * @version 0.1
 * @date 2024-03-07
 * @author Louis Munier
 * @author Tristan Bonato
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp" // Include the necessary header
#include "std_srvs/srv/trigger.hpp"

/**
 * @brief Class for ROS2 interface compatible with ROS2 Humble
 */
class RosInterfaceHumble : public rclcpp::Node {
public:
  /**
   * @brief Constructor.
   * @param robot_name Name of the robot.
   */
  explicit RosInterfaceHumble(const std::string &robot_name);
  Eigen::VectorXd getTwistKUL();
  // std::vector<double> getROSParam(const std::string &param_name);
  // void setROSParam(const std::string &param_name, const std::vector<double> &param_value);
  void getPointCloud();
  void publishNavmsg(const nav_msgs::msg::Path &path);
  void publishPolygon(const geometry_msgs::msg::PolygonStamped &polygonMsg);
  void publishSTL();
  void publishPoseFirst();
  nav_msgs::msg::Path convertFileToNavMsgsPath();
  geometry_msgs::msg::PolygonStamped convertPolygonToPolygonRosMsg();

  std::vector<std::vector<double>> convertNavPathToVectorVector(const nav_msgs::msg::Path &inputPath);

  // // State management
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> receiveState();
  void sendState(const std::vector<double> &data);
  void spinNode();
  Eigen::Affine3d getTransform(const std::string &target_frame, const std::string &source_frame);
  // bool getSharedControl();
  bool readyToWorkingPosition();
  // // Cartesian Twist and Pose management
  // void setCartesianTwist(const std::vector<double> &data);
  // void setDesiredDsTwist(const std::vector<double> &data);
  // void setError(const std::vector<double> &data);
  // void setForceFiltered(const std::vector<double> &data);
  // void setManipulability(const Eigen::Vector4d &data);
  // void setCartesianPose(const std::pair<Eigen::Quaterniond, Eigen::Vector3d>
  // &pairActualQuatPos);

  // // Wrench and FT sensor management
  // std::vector<double> receiveWrench();
  // bool callRobotiqService(uint8_t command_id);

  // void setWrenchBias(const std::vector<double> &wrenchBias);
  // bool checkFtConnection();

  // // VRPN Optitrack data
  // std::vector<double> receiveVrpnOptitrack();
  // Eigen::VectorXd receiveVrpnOptitrackTwist();

  // // Actuator spray management
  // void setActuatorSpray(bool data);

  // // Integration methods for KUL
  // bool isFirstPosReady();

  void setActualCartesianPoseEEF(const std::pair<Eigen::Quaterniond, Eigen::Vector3d> &data);
  void setActualCartesianTwistEEF(Eigen::VectorXd &data);

  bool isFirstPosReached();
  bool isShotcreteDone();
  bool isShotcreteReadyToBegin();

  void setShotcreteDone(bool &data);
  void setShotcreteBegin(bool &data);
  void setFirstPosReached(bool &data);

  // void setSharedControl(bool &data);

  std::vector<double> getWorkingPose();
  // Eigen::VectorXd getTwistKUL();
  // void publishFirstPoseAchieved(bool success);
  // void setCartesianPoseKUL(const std::pair<Eigen::Quaterniond,
  // Eigen::Vector3d> &data); void setCartesianTwistKUL(const Eigen::VectorXd
  // &data);

private:
  // Callback functions
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void twistKULCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void startShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stopShotcreteServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void firstPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void saveURDFToFile(std::string file_path, std::string param_name);

  // void sharedControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // void firstPoseKULCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  //   void shotcreteDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // publishers functions

  bool readSTL(const std::string &file_path, std::vector<geometry_msgs::msg::Point> &vertices);
  Eigen::Affine3d transformToEigen(const geometry_msgs::msg::TransformStamped &transform);

  std::string robotName_;
  int nJoint_;
  bool firstPosReceived_;
  bool firstPosReached_;
  bool firstTwistReceived_;
  bool shotcreteDone_;
  bool shotcreteBegin_;
  bool save_requested_;
  bool filterON_;

  // State variables
  std::vector<double> jointsPosition_;
  std::vector<double> jointsVelocity_;
  std::vector<double> jointsEffort_;
  std::vector<double> prevFilteredJointSpeed_; // Store the previous filtered joint speeds
  Eigen::VectorXd twistKUL_;
  std::vector<double> firstQuatPos_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;

  // // KUL variables

  // Eigen::VectorXd twist_shared_control_;
  // std::vector<double> desired_lin_speed_and_quat_;

  // service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initShotcreteService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startShotcreteService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopShotcreteService_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubStateJoints_;
  //   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubActuatorSpray_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubActualCartesianPoseEEF_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubActualCartesianTwistEEF_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_flat_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_final_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stl_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubShotcreteBegin_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubShotcreteDone_;
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubSharedControl_;

  // // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subFtSensor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subStateJoints_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subTwistKUL;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudTransformedCropSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subFirstPose_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sharedControlSub_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pos_array_sub_;

  std::mutex dataMutex_;
  std::string dataPath_;
  std::string reference_frame_;

  std::vector<geometry_msgs::msg::Point> vertices_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
