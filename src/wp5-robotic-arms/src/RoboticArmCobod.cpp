#include "RoboticArmCobod.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

#include "controllers/ControllerFactory.hpp"

using namespace controllers;
using namespace state_representation;
using namespace std;

// const double RoboticArmCobod::TOLERANCE = 1e-5;

RoboticArmCobod::RoboticArmCobod() {
  transformEigen_ = Eigen::Affine3d::Identity();
  robotName_ = "robetarme";
  string alternativeYamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/arm_robot_config.yaml";
  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/arm_robot_config.yaml";
  string nameYaml = "catkin_ws/config/arm_robot_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << nameYaml << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);
  YAML::Node robotNode = config[robotName_];

  pathUrdf_ = robotNode["path_urdf"].as<string>();
  tipLink_ = robotNode["tipLink"].as<string>();
  tipJoint_ = robotNode["tipJoint"].as<string>();
  baseLink_ = robotNode["reference_frame"].as<string>();
  jointNames_ = robotNode["controller_joint_names"].as<std::vector<std::string>>();
  referenceFrame_ = robotNode["reference_frame"].as<string>();
  nJoint_ = robotNode["numberJoint"].as<int>();
  originalHomeJoint = robotNode["home_position"].as<std::vector<double>>();
  twist_ = Eigen::VectorXd::Zero(6); // Reinitialize twist_ with size n, filled with 0.0

  model_ = make_unique<robot_model::Model>(robotName_, pathUrdf_);
  // Define the URDF and STL paths

  std::string package_path = std::string(WP5_ROBOTIC_ARMS_DIR);
  std::string urdf_path = package_path + "/urdf/robetarme.urdf";
  // std::string urdf_path =
  //     "/home/robetarme_user/ros2_ws/src/CobodPkgEPFL_ws/src/Cobod_RoBetArme_ROS2_Description/urdf/robetarme.urdf";
  // Define the obstacle paths using a vector
  // std::vector<std::string> obstacle_paths = {package_path
  //                                            + "/../wp5-planner/data/meshes/pointcloud_target_transformed.stl"};
  std::vector<std::string> obstacle_paths;

  initializeSelfAvoidance(urdf_path, obstacle_paths);
  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int maxNumberOfIterations = 1000;
  paramsIK_ = {damp, alpha, gamma, margin, tolerance, maxNumberOfIterations};
  initTracIK();
}
// RoboticArmCobod::~RoboticArmCobod() { delete robotGeoSolver_; }

vector<double> RoboticArmCobod::lowLevelController(
    const tuple<vector<double>, vector<double>, vector<double>>& stateJoints, const Eigen::VectorXd& twist) {
  const vector<double>& retrievedPosition = get<0>(stateJoints);
  double alpha = 0.5;
  twist_ = alpha * twist_ + (1 - alpha) * twist;
  vector<double> desiredJointSpeed = IRoboticArmBase::getInvertVelocities(retrievedPosition, twist_);
  return desiredJointSpeed;
}

vector<double> RoboticArmCobod::lowLevelControllerSF(
    const tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
    const Eigen::VectorXd& desiredTwist,
    const Eigen::VectorXd& deltaTwistFromWrench) {
  const vector<double>& retrievedPosition = get<0>(stateJoints);
  pair<Eigen::Quaterniond, Eigen::Vector3d> pairFK = getFKPinocchio(retrievedPosition);
  Eigen::VectorXd twist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd deltaTwistFromWrenchTransfrom =
      transformWrenchToBase_(deltaTwistFromWrench, pairFK.second, pairFK.first);
  twist = desiredTwist + deltaTwistFromWrenchTransfrom;

  // Save the vectors in the class-level containers
  if (desiredTwist != Eigen::VectorXd::Zero(6)) {
    savedDesiredTwists.push_back(desiredTwist);
    savedDeltaTwistFromWrenchTransfroms.push_back(deltaTwistFromWrenchTransfrom);
    savedTwists.push_back(twist);
  }
  double alpha = 0.5;
  twist_ = alpha * twist_ + (1 - alpha) * twist;
  vector<double> desiredJointSpeed = IRoboticArmBase::getInvertVelocities(retrievedPosition, twist_);
  return desiredJointSpeed;
}

bool RoboticArmCobod::saveTwistsToFiles(const std::string& desiredTwistFilename,
                                        const std::string& deltaTwistFilename,
                                        const std::string& twistFilename) {

  std::string packagePath = string(WP5_ROBOTIC_ARMS_DIR) + "/spaces/";
  std::ofstream desiredTwistFile(packagePath + desiredTwistFilename);
  std::ofstream deltaTwistFile(packagePath + deltaTwistFilename);
  std::ofstream twistFile(packagePath + twistFilename);

  // Check if all files opened correctly
  if (!desiredTwistFile || !deltaTwistFile || !twistFile) {
    std::cerr << "Error: Could not open one or more files." << std::endl;
    return false;
  }

  // Write desiredTwist to its file
  for (size_t i = 0; i < savedDesiredTwists.size(); ++i) {
    desiredTwistFile << savedDesiredTwists[i].transpose() << "\n";
  }

  // Write deltaTwistFromWrenchTransfrom to its file
  for (size_t i = 0; i < savedDeltaTwistFromWrenchTransfroms.size(); ++i) {
    deltaTwistFile << savedDeltaTwistFromWrenchTransfroms[i].transpose() << "\n";
  }

  // Write twist to its file
  for (size_t i = 0; i < savedTwists.size(); ++i) {
    twistFile << savedTwists[i].transpose() << "\n";
  }

  // Close all files
  desiredTwistFile.close();
  deltaTwistFile.close();
  twistFile.close();

  // std::cout << "Twists saved to files: " << desiredTwistFilename << ", " << deltaTwistFilename << ", " << twistFilename
  //           << std::endl;
  return true;
}

std::pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmCobod::getFKGeo(const std::vector<double>& jointPos) {
  throw std::runtime_error("Forward kinematics from GEO not supported for Cobod");
}

bool RoboticArmCobod::getIKGeo(const Eigen::Quaterniond& quaternion,
                               const Eigen::Vector3d& position,
                               std::vector<std::vector<double>>& jointPos) {
  throw std::runtime_error("Inverse kinematics from GEO not supported for Cobod");
}
