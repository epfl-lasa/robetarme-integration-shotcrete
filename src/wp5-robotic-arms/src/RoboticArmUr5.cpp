#include "RoboticArmUr5.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

#include "controllers/ControllerFactory.hpp"

using namespace controllers;
using namespace state_representation;
using namespace std;

const double RoboticArmUr5::TOLERANCE = 1e-5;

RoboticArmUr5::RoboticArmUr5() {
  transformEigen_ = Eigen::Affine3d::Identity();
  robotName_ = "ur5_robot";

  string alternativeYamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/arm_robot_config.yaml";
  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/arm_robot_config.yaml";
  string nameYaml = "ros2_ws/config/arm_robot_config.yaml";

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
  std::string pathUrdf_ = package_path + "/urdf/ur5_tool_sf.urdf";

  // Define the obstacle paths using a vector
  std::vector<std::string> obstacle_paths = {
      // package_path + "/../wp5-planner/data/meshes/pointcloud_target_transformed.stl",
      package_path + "/meshes/ridgeback.stl",
      package_path + "/meshes/ridgebackMount.stl"};

  initializeSelfAvoidance(pathUrdf_, obstacle_paths);

  initTracIK();

  ikGeoUp_ = false;
  robotGeoSolver_ = new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR5_H_MATRIX, UR5_P_MATRIX));
}
RoboticArmUr5::~RoboticArmUr5() { delete robotGeoSolver_; }

vector<double> RoboticArmUr5::lowLevelController(
    const tuple<vector<double>, vector<double>, vector<double>> &stateJoints, const Eigen::VectorXd &twist) {
  const vector<double> &retrievedPosition = get<0>(stateJoints);
  double alpha = 0;
  twist_ = alpha * twist_ + (1 - alpha) * twist;
  vector<double> desiredJointSpeed = IRoboticArmBase::getInvertVelocities(retrievedPosition, twist_);
  return desiredJointSpeed;
}

vector<double> RoboticArmUr5::lowLevelControllerSF(
    const tuple<vector<double>, vector<double>, vector<double>> &stateJoints,
    const Eigen::VectorXd &desiredTwist,
    const Eigen::VectorXd &deltaTwistFromWrench) {
  const vector<double> &retrievedPosition = get<0>(stateJoints);
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

bool RoboticArmUr5::saveTwistsToFiles(const std::string &desiredTwistFilename,
                                      const std::string &deltaTwistFilename,
                                      const std::string &twistFilename) {

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

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFKGeo(const vector<double> &jointPos) {
  // Offset to fix convention between trac-ik (the basic one to use) and ik-geo
  // solvers
  Eigen::Quaterniond offset = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

  // Arrays to hold the results of the forward kinematics
  array<double, 9> rotArr;
  array<double, 3> posArr;

  // Convert vector to array
  array<double, 6> jointPosArr;
  copy(jointPos.begin(), jointPos.end(), jointPosArr.begin());

  // Compute forward kinematics
  robotGeoSolver_->fk(jointPosArr, rotArr, posArr);

  Eigen::Vector3d posVector = Eigen::Vector3d::Map(posArr.data());
  Eigen::Quaterniond quaternion(Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotArr.data()));

  // Return the position and quaternion
  quaternion = quaternion * offset;
  return make_pair(move(quaternion), move(posVector));
}

bool RoboticArmUr5::getIKGeo(const Eigen::Quaterniond &quaternion,
                             const Eigen::Vector3d &position,
                             vector<vector<double>> &jointPos) {
  // Offset to fix convention between trac-ik (the basic one to use) and ik-geo
  // solvers
  Eigen::Quaterniond offset = Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5);

  uint totSolutions = 0;
  const uint MAX_REJECTIONS = 90; // percentage of rejected solutions
  double posVector[3] = {position.x(), position.y(), position.z()};

  double rotMatrixArray[9]{};
  Eigen::Matrix3d rotMatrix = (quaternion * offset).toRotationMatrix();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotMatrixArray, rotMatrix.rows(), rotMatrix.cols()) =
      rotMatrix;

  // Compute IK
  vector<ik_geo::Solution> ikSolutions;

  robotGeoSolver_->ik(rotMatrixArray, posVector, ikSolutions);

  // Check if the IK solver found valid solutions
  jointPos.clear();
  for (const auto &solution : ikSolutions) {
    vector<double> solutionVector(solution.q.begin(), solution.q.end());
    auto [quaternionSolution, positionSolution] = getFKGeo(solutionVector);

    bool isQuatValid = areQuaternionsEquivalent_(quaternion, quaternionSolution);
    bool isPosValid = arePositionsEquivalent_(position, positionSolution);

    if (isQuatValid && isPosValid) {
      jointPos.push_back(solutionVector);
    }
  }

  // Check wether the number of rejected solutions is not too high
  totSolutions = ikSolutions.size();
  if ((totSolutions < (MAX_REJECTIONS * jointPos.size() / 100)) || jointPos.empty()) {
    std::cout << "Too many solutions were rejected." << std::endl;
    return false;
  }

  return true;
}

bool RoboticArmUr5::areQuaternionsEquivalent_(const Eigen::Quaterniond &q1,
                                              const Eigen::Quaterniond &q2,
                                              double tolerance) const {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

// Function to check if two positions are equivalent
bool RoboticArmUr5::arePositionsEquivalent_(const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &p2,
                                            double tolerance) const {
  return (p1 - p2).norm() < tolerance;
}
