/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "IRoboticArmBase.h"

#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <typeinfo>

namespace py = pybind11;
using namespace std;
using namespace Eigen;

/**
 * @brief Mother class to create all the prototype fonctions needed in the
 * different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary
 * functions to control it.
 */
pair<Quaterniond, Vector3d> IRoboticArmBase::getFKPinocchio(const vector<double>& jointPos) const {
  Map<const VectorXd> posJointEigen(jointPos.data(), jointPos.size());
  state_representation::JointPositions nextJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::CartesianPose nextCartesianPose = model_->forward_kinematics(nextJoinState, tipLink_);

  Vector3d p1Prime = nextCartesianPose.get_position();
  Quaterniond q1Prime = nextCartesianPose.get_orientation();

  return make_pair(q1Prime, p1Prime);
}

pair<Quaterniond, Vector3d> IRoboticArmBase::getFKPinocchio(const vector<double>& jointPos,
                                                            const string& tipLink) const {
  Map<const VectorXd> posJointEigen(jointPos.data(), jointPos.size());
  state_representation::JointPositions nextJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::CartesianPose nextCartesianPose = model_->forward_kinematics(nextJoinState, tipLink);

  Vector3d p1Prime = nextCartesianPose.get_position();
  Quaterniond q1Prime = nextCartesianPose.get_orientation();

  return make_pair(q1Prime, p1Prime);
}

bool IRoboticArmBase::setTransformEigen(const Eigen::Affine3d& transform) {
  if (transform.matrix().allFinite()) {
    transformEigen_ = transform;
    return true;
  } else {
    std::cerr << "Invalid transform: contains non-finite values." << std::endl;
    return false;
  }
}

vector<double> IRoboticArmBase::getIKPinocchio(const Quaterniond& quaternion,
                                               const Vector3d& position,
                                               const vector<double>& jointPos) const {
  vector<double> positionJointNext(nJoint_);

  Map<const VectorXd> posJointEigen(jointPos.data(), jointPos.size());
  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);

  state_representation::CartesianPose cartesianPose =
      state_representation::CartesianPose(robotName_, position, quaternion, referenceFrame_);

  state_representation::JointPositions nextJoinState =
      model_->inverse_kinematics(cartesianPose, actualJoinState, paramsIK_, referenceFrame_);

  VectorXd positionJointNext_eigen = nextJoinState.data();

  Eigen::VectorXd::Map(&positionJointNext[0], nJoint_) = positionJointNext_eigen;
  return positionJointNext;
}

VectorXd IRoboticArmBase::getTwistFromJointState(const vector<double>& posJoint,
                                                 const vector<double>& speedJoint) const {
  MatrixXd jacMatrix = getJacobian(posJoint);
  VectorXd speedJointEigen(nJoint_);

  for (int i = 0; i < nJoint_; ++i) {
    speedJointEigen(i) = speedJoint[i];
  }

  VectorXd twistLinearAngular = jacMatrix * speedJointEigen;

  return twistLinearAngular;
}

MatrixXd IRoboticArmBase::getJacobian(const vector<double>& jointPos) const {
  Map<const VectorXd> posJointEigen(jointPos.data(), jointPos.size());
  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::Jacobian jacobianObject = model_->compute_jacobian(actualJoinState);
  MatrixXd jacobian = jacobianObject.data();

  return jacobian;
}

vector<double> IRoboticArmBase::getInvertVelocities(const vector<double>& jointPos, const VectorXd& speedEigen) const {
  vector<double> speedJointNext(nJoint_);

  Vector3d angularVelocity(3);
  Vector3d linearVelocity(3);
  linearVelocity << speedEigen(0), speedEigen(1), speedEigen(2);
  angularVelocity << speedEigen(3), speedEigen(4), speedEigen(5);

  Map<const VectorXd> posJointEigen(jointPos.data(), jointPos.size());

  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::CartesianTwist nextPostwist =
      state_representation::CartesianTwist(robotName_, linearVelocity, angularVelocity, referenceFrame_);
  state_representation::JointVelocities nextJoinStateSpeed =
      model_->inverse_velocity(nextPostwist, actualJoinState, tipJoint_);
  VectorXd speedJointNext_eigen = nextJoinStateSpeed.data();
  for (int i = 0; i < nJoint_; ++i) {
    speedJointNext[i] = speedJointNext_eigen(i);
  }

  return speedJointNext;
}

VectorXd IRoboticArmBase::computeSingularValues(const MatrixXd& J) const {
  JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
  return svd.singularValues();
}

VectorXd IRoboticArmBase::transformWrenchToBase_(const VectorXd& wrench_end_effector,
                                                 const Vector3d& position_end_effector,
                                                 const Quaterniond& orientation_end_effector_to_base) const {
  // Extract rotation quaternion from end effector to base transformation
  Matrix3d rotation_matrix = orientation_end_effector_to_base.toRotationMatrix();
  // Extract force and torque from wrench
  Vector3d force_end_effector = wrench_end_effector.head<3>();
  Vector3d torque_end_effector = wrench_end_effector.tail<3>();
  // Transform wrench to the base frame
  Vector3d force_base = rotation_matrix * force_end_effector;
  Vector3d torque_base = rotation_matrix * torque_end_effector;
  // Combine transformed force and torque into a single vector
  VectorXd wrench_base(6);
  wrench_base << force_base(0), force_base(1), force_base(2), torque_base(0), torque_base(1), torque_base(2);
  return wrench_base;
}

// function for the inverse kinematic
// ------------------------------------------------------------------------

pair<int, vector<double>> IRoboticArmBase::getTracIK(const vector<double>& actualJoint,
                                                     const vector<double>& vectorQuatPos) const {

  //Inverse kinematics trac-IK
  KDL::JntArray NextJointTask;
  KDL::JntArray actualJointTask;

  VectorXd pos_joint_actual_eigen(nJoint_);
  for (int i = 0; i < nJoint_; ++i) {
    pos_joint_actual_eigen(i) = actualJoint[i];
  }
  actualJointTask.data = pos_joint_actual_eigen;

  KDL::Vector Vec(vectorQuatPos[4], vectorQuatPos[5], vectorQuatPos[6]);

  Quaterniond q(vectorQuatPos[3], vectorQuatPos[0], vectorQuatPos[1], vectorQuatPos[2]); // w x y z
  q.normalize();
  KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
  KDL::Frame NextJointCartesian(Rot, Vec);
  int rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);

  if (rc < 0) {
    cout << "no inverse kinematic found" << endl;
  }

  VectorXd posJointNextEigen = NextJointTask.data;
  vector<double> posJointNext(nJoint_, 0.0);

  Eigen::Map<Eigen::VectorXd>(posJointNext.data(), nJoint_) = posJointNextEigen;

  pair<int, vector<double>> myPair = make_pair(rc, posJointNext);
  return myPair;
}
// Helper function to check if two joint configurations are different
bool areSolutionsDifferent(const vector<double>& sol1, const vector<double>& sol2, double tolerance) {
  if (sol1.size() != sol2.size())
    return false;
  double diff = 0.0;
  for (size_t i = 0; i < sol1.size(); ++i) {
    diff += fabs(sol1[i] - sol2[i]);
  }
  return diff > tolerance;
}

// Function to convert Eigen::VectorXd limits into vector of pairs
std::vector<std::pair<double, double>> IRoboticArmBase::getJointLimits() const {
  std::vector<std::pair<double, double>> jointLimits;
  VectorXd lowerPositionLimitEigen = model_->get_pinocchio_model().lowerPositionLimit;
  VectorXd upperPositionLimitEigen = model_->get_pinocchio_model().upperPositionLimit;
  // Ensure both vectors have the same size
  if (lowerPositionLimitEigen.size() != upperPositionLimitEigen.size()) {
    throw std::invalid_argument("Lower and upper limits must have the same size.");
  }

  // Populate the vector with pairs
  for (int i = 0; i < lowerPositionLimitEigen.size(); ++i) {
    jointLimits.emplace_back(lowerPositionLimitEigen[i], upperPositionLimitEigen[i]);
  }

  return jointLimits;
}

std::pair<bool, std::vector<std::vector<double>>> IRoboticArmBase::getUniqueTracIKSolutions(
    const std::vector<double>& actualJoint,
    const std::vector<double>& vectorQuatPos,
    int n = 5,
    double tolerance = 1e-3) const {

  std::vector<std::vector<double>> iksFirstPosition; // Container for solutions
  bool foundSolutions = false;                       // Flag to indicate success

  // Get joint limits (replace with actual joint limits of the robotic arm)
  std::vector<std::pair<double, double>> jointLimits = getJointLimits();

  std::random_device rd;
  std::mt19937 gen(rd());

  int attempts = 0;
  const int maxAttempts = 1000; // Prevent infinite loop

  // First attempt uses the actual joint configuration
  std::vector<double> testJointConfig = actualJoint;

  while (iksFirstPosition.size() < n && attempts < maxAttempts) {
    if (!iksFirstPosition.empty()) {
      // Generate a random joint configuration within the limits
      testJointConfig.clear();
      for (const auto& limit : jointLimits) {
        std::uniform_real_distribution<double> dist(limit.first, limit.second);
        testJointConfig.push_back(dist(gen));
      }
    }

    auto ikSolution = getTracIK(testJointConfig, vectorQuatPos);

    if (ikSolution.first >= 0) { // Valid solution found
      bool isUnique = true;

      for (const auto& sol : iksFirstPosition) {
        if (!areSolutionsDifferent(ikSolution.second, sol, tolerance)) {
          isUnique = false;
          break;
        }
      }

      if (isUnique) {
        iksFirstPosition.push_back(ikSolution.second);
      }
    }

    ++attempts;
  }

  foundSolutions = !iksFirstPosition.empty();
  return {foundSolutions, iksFirstPosition};
}

bool IRoboticArmBase::checkIkGeoUp() const { return (ikGeoUp_); }

// Helper function to load a URDF from file
std::string loadURDF(const std::string& file_path) {
  std::ifstream urdf_file(file_path);
  if (!urdf_file.is_open()) {
    throw std::runtime_error("Unable to open URDF file: " + file_path);
  }

  std::stringstream urdf_stream;
  urdf_stream << urdf_file.rdbuf();
  return urdf_stream.str();
}

void IRoboticArmBase::initTracIK() {
  // Define paths for the YAML configuration file
  std::string alternativeYamlPath = std::string(WP5_ROBOTIC_ARMS_DIR) + "/config/tracIk_config.yaml";
  std::string yamlPath = std::string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/tracIk_config.yaml";
  std::string nameYaml = "catkin_ws/config/tracIk_config.yaml";

  // Check if the alternative YAML file exists
  std::ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    std::cout << "Using general YAML file: " << nameYaml << std::endl;
  } else {
    yamlPath = alternativeYamlPath;
    std::cout << "Using local YAML file: " << yamlPath << std::endl;
  }

  // Load parameters from YAML file
  YAML::Node yaml = YAML::LoadFile(yamlPath);
  YAML::Node config = yaml["tracIK"];

  // Get the solve type from the YAML file
  std::string solveTypeStr = config["solve_type"].as<std::string>();

  // Convert the solve type string to the corresponding enum value
  TRAC_IK::SolveType solveType;
  if (solveTypeStr == "Distance") {
    solveType = TRAC_IK::Distance;
  } else if (solveTypeStr == "Speed") {
    solveType = TRAC_IK::Speed;
  } else {
    std::cout << "Handle unrecognized solve types: set Distance as default value" << std::endl;
    solveType = TRAC_IK::Distance;
  }

  double error = config["error"].as<double>();
  double timeoutInSecs = config["timeoutInSecs"].as<double>();
  // std::string nameParamUrdf = config["robot_description_param"].as<double>();
  std::string nameParamUrdf = "robot_description";

  // Create a ROS 2 node shared pointer
  std::string urdf_content = loadURDF(pathUrdf_);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("trac_ik_node");
  node->declare_parameter<std::string>(nameParamUrdf, urdf_content);
  node->set_parameter(rclcpp::Parameter(nameParamUrdf, urdf_content));

  // Construct the TRAC-IK object
  ikSolver = new TRAC_IK::TRAC_IK(node,          // ROS 2 node pointer
                                  baseLink_,     // Base link
                                  tipLink_,      // Tip link
                                  nameParamUrdf, // URDF parameter name
                                  timeoutInSecs, // Timeout in seconds
                                  error,         // Error tolerance
                                  solveType      // Solve type
  );

  // Validate the KDL chain
  bool valid = ikSolver->getKDLChain(chain_);
  if (!valid) {
    std::cout << "There was no valid KDL chain found" << std::endl;
  }
}

void IRoboticArmBase::initializeSelfAvoidance(const std::string& urdf_path,
                                              const std::vector<std::string>& obstacle_stl_paths) {
  // Check if Python interpreter is already initialized
  if (!Py_IsInitialized()) {
    // std::cout << "Python interpreter not running. Initializing..." << std::endl;
    py::initialize_interpreter(); // Initialize Python interpreter
  } else {
    // std::cout << "Python interpreter is already running." << std::endl;
  }

  try {
    // Add the `scripts` directory to Python's sys.path
    std::string pathPackage = std::string(WP5_ROBOTIC_ARMS_DIR) + "/scripts";
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")(pathPackage);

    // Import the self_avoidance module and initialize the SelfAvoidance instance
    py::module planner_module = py::module::import("self_avoidance");
    py::object SelfAvoidance = planner_module.attr("SelfAvoidance");

    // Create SelfAvoidance instance
    self_avoidance_instance_ =
        std::make_unique<py::object>(SelfAvoidance(urdf_path, py::none(), false, obstacle_stl_paths));

  } catch (const py::error_already_set& e) {
    std::cerr << "Python error: " << e.what() << std::endl;
    throw;
  }
}

// Use the initialized SelfAvoidance instance in getFreePath
std::vector<std::vector<double>> IRoboticArmBase::getFreePath(const std::vector<double>& start_configuration,
                                                              const std::vector<double>& target_configuration) const {
  try {
    // Call the `plan_path` method on the existing SelfAvoidance instance
    py::object path = self_avoidance_instance_->attr("plan_path")(start_configuration, target_configuration);

    // Check if a path was found
    if (path.is_none()) {
      std::cout << "No collision-free path could be found." << std::endl;
      return {};
    }

    // Convert the Python list of lists to a C++ vector of vectors
    return path.cast<std::vector<std::vector<double>>>();

  } catch (py::error_already_set& e) {
    std::cerr << "Python error: " << e.what() << std::endl;
    return {};
  }
}
// Use the initialized SelfAvoidance instance in getFreePath
bool IRoboticArmBase::checkCollision(const std::vector<double>& jointConfig) const {
  try {
    // Call the `plan_path` method on the existing SelfAvoidance instance
    py::object path = self_avoidance_instance_->attr("check_collision_with_objects")(jointConfig);

    // Convert the Python list of lists to a C++ vector of vectors
    return path.cast<bool>();

  } catch (py::error_already_set& e) {
    std::cerr << "Python error: " << e.what() << std::endl;
    return {};
  }
}